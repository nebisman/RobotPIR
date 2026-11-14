# ==============================================================================
# INFRAESTRUCTURA MULTIPLATAFORMA (WINDOWS / LINUX)
# Archivo: lqr_pipeline.jl
# ==============================================================================

using Printf
using LibSerialPort

# ------------------------------------------------------------------------------
# 1. IDENTIFICACIÓN AUTOMÁTICA DE PUERTO SERIE
# ------------------------------------------------------------------------------
function find_port()
    known_vid_pid = Set{Tuple{UInt16,UInt16}}([
        (0x10C4, 0xEA60),  # Silicon Labs CP2102/CP2104
        (0x1A86, 0x7523),  # WCH CH340/CH341
        (0x1A86, 0x7522),  # WCH CH340 (robot STM32)
        (0x1A86, 0x55D4),  # WCH CH9102
        (0x0403, 0x6001),  # FTDI FT232
        (0x303A, 0x1001),  # Espressif USB JTAG/serial
    ])
    known_keywords = ("cp210", "ch340", "ch9102", "wch", "silicon labs",
                       "espressif", "esp32", "usb-serial", "usb serial", "uart", "ttyusb", "ttyacm")

    candidates = String[]
    ports = LibSerialPort.sp_list_ports()
    try
        i = 1
        while true
            port = unsafe_load(ports, i)
            port == C_NULL && break
            if LibSerialPort.sp_get_port_transport(port) == LibSerialPort.SP_TRANSPORT_USB
                manufacturer = LibSerialPort.sp_get_port_usb_manufacturer(port)
                product      = LibSerialPort.sp_get_port_usb_product(port)
                description  = LibSerialPort.sp_get_port_description(port)
                vid, pid     = LibSerialPort.sp_get_port_usb_vid_pid(port)

                text = lowercase(join((manufacturer, product, description), " "))
                matched = (UInt16(vid), UInt16(pid)) in known_vid_pid ||
                          any(kw -> occursin(kw, text), known_keywords)
                matched && push!(candidates, LibSerialPort.sp_get_port_name(port))
            end
            i += 1
        end
    finally
        LibSerialPort.sp_free_port_list(ports)
    end

    if isempty(candidates) 
        error("No se encontró ningún puerto USB-serial compatible conectado.")
    elseif length(candidates) > 1
        error("Se encontraron múltiples puertos candidatos: " * join(candidates, ", ") * ". Especifique el puerto manualmente.")
    end  
    return candidates[1]
end

function update_tasks_com_port(port::String)
    curr_dir = @__DIR__
    found_paths = String[]
    
    for _ in 1:4
        p = normpath(joinpath(curr_dir, ".vscode", "tasks.json"))
        if isfile(p) && !(p in found_paths)
            push!(found_paths, p)
        end
        curr_dir = dirname(curr_dir)
    end

    if isempty(found_paths)
        return
    end

    for tasks_path in found_paths
        content = read(tasks_path, String)
        # Expresión regular compatible con COMx (Windows) y /dev/ttyUSBx (Linux)
        new_content = replace(content, r"(COM\d+|/dev/tty[A-Za-z0-9]+)"i => port)
        
        if content != new_content
            open(tasks_path, "w") do io
                write(io, new_content)
            end
            println("✅ Puerto $port asignado con éxito en:\n   $tasks_path")
        end
    end
end

# ------------------------------------------------------------------------------
# 2. GENERACIÓN DE C HEADERS (lqr_gain_mode0/1/2.h y lqr_mode.h)
# ------------------------------------------------------------------------------
# Modos de control del firmware (APP/app_control.c):
#
#   Modo 0 - Realimentación de estados (control_lqr_mode0.jl, asignacion_polos.jl)
#            x = [x, v, θ, θ̇, ψ, ψ̇],  u = -K (x - x_ref),  K de 2×6
#            Usa las referencias ref_pos y ref_yaw
#
#   Modo 1 - Acción integral sobre la velocidad lineal (control_lqr_mode1.jl)
#            x = [v, θ, θ̇, ψ, ψ̇],  z = ∫(ref_vel - v) dt,  u = -K [x; z],  K de 2×6
#            Usa las referencias ref_vel y ref_yaw
#
#   Modo 2 - Velocidad angular de yaw con acción integral (control_lqr_mode2.jl)
#            x = [x, v, θ, θ̇, ψ̇],  z = ∫(ref_wd - ψ̇) dt,  u = -(K (x - x_ref) + Ki z),  K de 2×6
#            Usa las referencias ref_pos y ref_wd
#
# Referencias: una sola de cada una, compartida por todos los modos, con el
# mismo nombre y las mismas unidades que en app_control.c:
#   ref_pos (m), ref_vel (m/s), ref_yaw (rad), ref_wd (rad/s)
#   (un ángulo en grados se pasa como deg2rad(...))
#
# Cada vez que se programa un modo se escriben dos archivos en APP/:
#   lqr_gain_mode<N>.h : ganancias del modo N, con sufijo _N
#                        (K11_0_LQR, ..., K12_1_LQR, Ki1_1_LQR, ..., K16_2_LQR, Ki1_2_LQR)
#   lqr_mode.h         : LQR_MODE (modo con el que arranca el robot) y las
#                        referencias REFPOS_LQR, REFVEL_LQR, REFYAW_LQR, REFWD_LQR
# Los headers de ganancias de los otros modos no se modifican. El firmware
# compila siempre las ganancias de todos los modos, así que puede cambiar de
# modo en ejecución con Set_Control_Mode(0, 1 o 2).

const MODOS_LQR = Dict(
    0 => "realimentación de estados",
    1 => "acción integral de velocidad lineal",
    2 => "velocidad angular de yaw con acción integral",
)

# Tamaño de K que espera cada modo (columnas en el orden de su vector de estado)
const DIM_K = Dict(0 => (2, 6), 1 => (2, 6), 2 => (2, 6))

# Encabezado común de los archivos .h (solo ASCII: Keil muestra los .c/.h como GBK).
function _cabecera_c!(io, guard, lineas)
    println(io, "/* ======================================================================= */")
    for l in lineas
        println(io, "/* ", rpad(l, 71), " */")
    end
    println(io, "/* ======================================================================= */")
    println(io, "#ifndef $guard")
    println(io, "#define $guard\n")
end

_define(io, nombre, valor, comentario) =
    @printf(io, "#define %-13s %16.10f  // %s\n", nombre, valor, comentario)

function _header_modo0(K)
    io = IOBuffer()
    _cabecera_c!(io, "__LQR_GAIN_MODE0_H", [
        "lqr_gain_mode0.h - MODO 0: realimentacion de estados",
        "ARCHIVO GENERADO AUTOMATICAMENTE DESDE JULIA (set_ss_controller)",
        "NO EDITAR MANUALMENTE - ejecutar control_lqr_mode0.jl",
        "",
        "u = -K (x - x_ref),  x = [x, v, pitch, pitch_rate, yaw, yaw_rate]",
        "Sufijo _0: ganancias del modo 0 (conviven con las _1 del modo 1)",
        "Las referencias estan en lqr_mode.h",
    ])
    estados = ("Posicion x", "Velocidad lineal v", "Angulo pitch",
               "Vel. angular pitch", "Angulo yaw", "Vel. angular yaw")
    for (i, rueda) in enumerate(("izquierda", "derecha"))
        println(io, "// Rueda $rueda (fila $i de K)")
        for j in 1:6
            _define(io, "K$(i)$(j)_0_LQR", K[i, j], estados[j])
        end
        println(io)
    end
    println(io, "#endif /* __LQR_GAIN_MODE0_H */")
    return String(take!(io))
end

function _header_modo1(K)
    io = IOBuffer()
    _cabecera_c!(io, "__LQR_GAIN_MODE1_H", [
        "lqr_gain_mode1.h - MODO 1: accion integral de velocidad lineal",
        "ARCHIVO GENERADO AUTOMATICAMENTE DESDE JULIA (set_ss_controller)",
        "NO EDITAR MANUALMENTE - ejecutar control_lqr_mode1.jl",
        "",
        "u = -(K x + Ki z),  x = [v, pitch, pitch_rate, yaw, yaw_rate]",
        "z = integral(ref_vel - v)",
        "Los indices Kij siguen el estado completo del modo 0 (j=2: v, ..., 6);",
        "no hay K11/K21 porque el modo 1 no realimenta la posicion.",
        "Sufijo _1: ganancias del modo 1 (conviven con las _0 del modo 0)",
        "Las referencias estan en lqr_mode.h",
    ])
    estados = ("Velocidad lineal v", "Angulo pitch", "Vel. angular pitch",
               "Angulo yaw", "Vel. angular yaw")
    for (i, rueda) in enumerate(("izquierda", "derecha"))
        println(io, "// Rueda $rueda (fila $i de K)")
        for j in 1:5
            _define(io, "K$(i)$(j + 1)_1_LQR", K[i, j], estados[j])
        end
        _define(io, "Ki$(i)_1_LQR", K[i, 6], "Integral del error de velocidad")
        println(io)
    end
    println(io, "#endif /* __LQR_GAIN_MODE1_H */")
    return String(take!(io))
end

function _header_modo2(K)
    io = IOBuffer()
    _cabecera_c!(io, "__LQR_GAIN_MODE2_H", [
        "lqr_gain_mode2.h - MODO 2: velocidad de yaw con accion integral",
        "ARCHIVO GENERADO AUTOMATICAMENTE DESDE JULIA (set_ss_controller)",
        "NO EDITAR MANUALMENTE - ejecutar control_lqr_mode2.jl",
        "",
        "u = -(K (x - x_ref) + Ki z),  x = [x, v, pitch, pitch_rate, yaw_rate]",
        "z = integral(ref_wd - yaw_rate)",
        "Los indices Kij siguen el estado completo del modo 0 (j=6: yaw_rate);",
        "no hay K15/K25 porque el modo 2 no realimenta el angulo de yaw.",
        "Sufijo _2: ganancias del modo 2 (conviven con las de los otros modos)",
        "Las referencias estan en lqr_mode.h",
    ])
    estados = ("Posicion x", "Velocidad lineal v", "Angulo pitch",
               "Vel. angular pitch", "Vel. angular yaw")
    indices = (1, 2, 3, 4, 6)     # columna de K -> indice j del estado completo
    for (i, rueda) in enumerate(("izquierda", "derecha"))
        println(io, "// Rueda $rueda (fila $i de K)")
        for (c, j) in enumerate(indices)
            _define(io, "K$(i)$(j)_2_LQR", K[i, c], estados[c])
        end
        _define(io, "Ki$(i)_2_LQR", K[i, 6], "Integral del error de vel. yaw")
        println(io)
    end
    println(io, "#endif /* __LQR_GAIN_MODE2_H */")
    return String(take!(io))
end

function _header_modo_activo(mode, ref_pos, ref_vel, ref_yaw, ref_wd)
    io = IOBuffer()
    _cabecera_c!(io, "__LQR_MODE_H", [
        "lqr_mode.h - MODO INICIAL Y REFERENCIAS",
        "ARCHIVO GENERADO AUTOMATICAMENTE DESDE JULIA (set_ss_controller)",
        "LQR_MODE: modo con el que arranca el robot (el ultimo programado);",
        "en ejecucion se cambia con Set_Control_Mode() (app_control.c).",
        "Las referencias son las mismas para todos los modos.",
    ])
    @printf(io, "#define LQR_MODE %d  // 0: estados, 1: integral de velocidad, 2: vel. de yaw con integral\n\n", mode)
    println(io, "// Referencias (variables ref_pos, ref_vel, ref_yaw, ref_wd de app_control.c)")
    _define(io, "REFPOS_LQR", ref_pos, "ref_pos: posicion x (m), modos 0 y 2")
    _define(io, "REFVEL_LQR", ref_vel, "ref_vel: velocidad lineal (m/s), modo 1")
    _define(io, "REFYAW_LQR", ref_yaw, "ref_yaw: angulo yaw (rad), modos 0 y 1")
    _define(io, "REFWD_LQR", ref_wd, "ref_wd: velocidad angular yaw (rad/s), modo 2")
    println(io, "\n#endif /* __LQR_MODE_H */")
    return String(take!(io))
end

function write_controller(K; mode = 0, ref_pos = 0.0, ref_vel = 0.0, ref_yaw = 0.0, ref_wd = 0.0,
                          app_dir = nothing)
    haskey(MODOS_LQR, mode) ||
        error("Modo $mode no válido. Modos disponibles: " *
              join(("$k ($(MODOS_LQR[k]))" for k in sort(collect(keys(MODOS_LQR)))), ", ") * ".")
    m_esp, n_esp = DIM_K[mode]
    size(K) == (m_esp, n_esp) ||
        error("En el modo $mode la matriz K debe ser $(m_esp)×$(n_esp); se recibió $(size(K, 1))×$(size(K, 2)).")

    if isnothing(app_dir)
        app_dir = normpath(joinpath(@__DIR__, "..", "APP"))
    end
    isdir(app_dir) || error("No se encontró la carpeta del firmware: $app_dir")

    abs(ref_yaw) > 2pi &&
        @warn "ref_yaw = $ref_yaw rad es más de una vuelta. El yaw va en radianes, como en el firmware; si lo tiene en grados use deg2rad($ref_yaw)."
    abs(ref_wd) > 2pi &&
        @warn "ref_wd = $ref_wd rad/s es más de una vuelta por segundo. Va en rad/s, como en el firmware; si la tiene en grados/s use deg2rad($ref_wd)."

    # Se generan los textos completos antes de escribir: si algo falla,
    # los headers existentes quedan intactos.
    contenido = (_header_modo0, _header_modo1, _header_modo2)[mode + 1](K)
    modo_y_refs = _header_modo_activo(mode, ref_pos, ref_vel, ref_yaw, ref_wd)

    archivo_gain = joinpath(app_dir, "lqr_gain_mode$(mode).h")
    archivo_modo = joinpath(app_dir, "lqr_mode.h")
    write(archivo_gain, contenido)
    write(archivo_modo, modo_y_refs)

    println("✅ Modo $mode ($(MODOS_LQR[mode])) generado:")
    println("   Ganancias:                $archivo_gain")
    println("   Modo inicial y referencias: $archivo_modo")
    @printf("   ref_pos = %g m, ref_vel = %g m/s, ref_yaw = %g rad, ref_wd = %g rad/s\n",
            ref_pos, ref_vel, ref_yaw, ref_wd)
    return archivo_gain
end

# ------------------------------------------------------------------------------
# 3. COMPILACIÓN Y FLASHEO MULTIPLATAFORMA
# ------------------------------------------------------------------------------

# Localiza cbuild, cmake, ninja y armclang y construye el entorno que necesita
# cbuild. Cuando Julia no se lanza desde una terminal con el entorno de Arm
# activado (Arm Environment Manager / vcpkg), estas herramientas no están en el
# PATH y cbuild falla con "unable to find a build program corresponding to Ninja".
function arm_build_env()
    tools = ("cbuild", "cmake", "ninja", "armclang")
    ext   = Sys.iswindows() ? ".exe" : ""
    found = Dict{String,String}()

    for t in tools
        p = Sys.which(t)
        isnothing(p) || (found[t] = p)
    end

    search_roots = [
        joinpath(homedir(), ".vcpkg", "artifacts"),   # herramientas de vcpkg-configuration.json
        joinpath(homedir(), ".vscode", "extensions"),
    ]
    for search_root in search_roots
        length(found) == length(tools) && break
        isdir(search_root) || continue
        for (root, _, files) in walkdir(search_root)
            lfiles = lowercase.(files)
            for t in tools
                if !haskey(found, t) && (t * ext) in lfiles
                    found[t] = normpath(joinpath(root, t * ext))
                end
            end
            length(found) == length(tools) && break
        end
    end

    missing_tools = [t for t in tools if !haskey(found, t)]
    if !isempty(missing_tools)
        error("No se localizaron: " * join(missing_tools, ", ") *
              ". Abra el proyecto en VS Code con Arm Keil Studio para que instale las herramientas de vcpkg-configuration.json.")
    end

    sep      = Sys.iswindows() ? ";" : ":"
    bin_dirs = unique([dirname(found[t]) for t in tools])
    env      = Dict{String,String}("PATH" => join(bin_dirs, sep) * sep * get(ENV, "PATH", ""))

    # cbuild ubica el compilador Arm mediante AC6_TOOLCHAIN_<mayor>_<menor>_<parche>
    armclang_dir = dirname(found["armclang"])
    m = match(r"(\d+)\.(\d+)\.(\d+)", basename(dirname(armclang_dir)))
    if !isnothing(m)
        var = "AC6_TOOLCHAIN_" * join(m.captures, "_")
        haskey(ENV, var) || (env[var] = joinpath(armclang_dir, ""))
    end

    return found["cbuild"], env
end

function compile_and_flash(puerto::String)
    dir_user  = @__DIR__                             # STM32_code/USER
    dir_stm32 = normpath(joinpath(dir_user, ".."))   # STM32_code
    
    # Selección de ejecutable de flasheo según el sistema operativo
    flasher_name = Sys.iswindows() ? "stm32flash.exe" : (Sys.islinux() ? "stm32flash_linux" : "stm32flash")
    flasher      = normpath(joinpath(dir_user, "tools", flasher_name))

    if !isfile(flasher)
        flasher_sys = Sys.which("stm32flash")
        flasher = isnothing(flasher_sys) ? flasher : flasher_sys
    end

    if !isfile(flasher)
        error("No se encontró '$flasher_name' en: $flasher")
    end

    # Dar permisos de ejecución en Linux
    if Sys.islinux()
        run(`chmod +x $flasher`)
    end

    println("\n🔨 1. Compilando el firmware C con Arm Keil Studio / CMSIS-Toolbox...")

    cbuild_path, build_env = arm_build_env()

    proj_file = ""
    for (root, dirs, files) in walkdir(dir_stm32)
        for f in files
            lf = lowercase(f)
            if endswith(lf, ".csolution.yml") || endswith(lf, ".csolution.yaml") || endswith(lf, ".cprj")
                proj_file = normpath(joinpath(root, f))
                break
            end
        end
        !isempty(proj_file) && break
    end

    println("   Usando CMSIS-Build: $cbuild_path")
    cd(dir_stm32) do
        if !isempty(proj_file)
            run(addenv(`$cbuild_path $proj_file --packs`, build_env))
        else
            run(addenv(`$cbuild_path --packs`, build_env))
        end
    end

    println("\n🔍 2. Verificando ejecutable y generando binario .hex...")
    hex_file = ""
    axf_file = ""

    search_dirs = [
        joinpath(dir_user, "out"),
        joinpath(dir_stm32, "out"),
        normpath(joinpath(dir_user, "..", ".."))
    ]

    for s_dir in search_dirs
        if isdir(s_dir)
            for (root, dirs, files) in walkdir(s_dir)
                for f in files
                    lf = lowercase(f)
                    if endswith(lf, ".hex") && isempty(hex_file)
                        hex_file = normpath(joinpath(root, f))
                    elseif (endswith(lf, ".axf") || endswith(lf, ".elf")) && isempty(axf_file)
                        axf_file = normpath(joinpath(root, f))
                    end
                end
            end
        end
    end

    if isempty(hex_file) && !isempty(axf_file)
        println("   Archivo compilado detectado: $axf_file")
        fromelf_exe  = Sys.iswindows() ? "fromelf.exe" : "fromelf"
        fromelf_path = Sys.which("fromelf")
        
        if isnothing(fromelf_path)
            vcpkg_dir = joinpath(homedir(), ".vcpkg", "artifacts")
            if isdir(vcpkg_dir)
                for (root, dirs, files) in walkdir(vcpkg_dir)
                    if fromelf_exe in lowercase.(files)
                        fromelf_path = normpath(joinpath(root, fromelf_exe))
                        break
                    end
                end
            end
        end

        if !isnothing(fromelf_path) && isfile(fromelf_path)
            hex_file = replace(axf_file, r"\.(axf|elf)$"i => ".hex")
            println("   Convertiendo .axf ➔ .hex usando fromelf: $fromelf_path")
            run(`$fromelf_path --i32 --output=$hex_file $axf_file`)
        else
            # Si no hay fromelf en Linux, intentar con arm-none-eabi-objcopy
            objcopy = Sys.which("arm-none-eabi-objcopy")
            if !isnothing(objcopy)
                hex_file = replace(axf_file, r"\.(axf|elf)$"i => ".hex")
                println("   Convertiendo .axf/.elf ➔ .hex usando objcopy: $objcopy")
                run(`$objcopy -O ihex $axf_file $hex_file`)
            else
                error("No se encontró 'fromelf' ni 'arm-none-eabi-objcopy' para generar el archivo .hex.")
            end
        end
    end

    println("\n⚡ 3. Flasheando binario .hex a la STM32 en el puerto $puerto...")
    println("   Herramienta: $flasher")
    println("   Archivo .hex: $hex_file")

    if isempty(hex_file) || !isfile(hex_file)
        error("No se pudo obtener el archivo .hex para el flasheo.")
    end

    # Secuencia DTR/RTS para entrar al bootloader (DTR -> NRST, RTS -> BOOT0).
    # En Windows cada cambio de señal es lento y los 'rts' repetidos actúan como
    # pausa. En Linux/macOS son instantáneos: el pulso de reset resulta demasiado
    # corto ("Failed to init device"), así que se usan pausas explícitas con ','
    # (100 ms cada una). Al salir: BOOT0 en bajo y pulso de reset para arrancar.
    gpio_seq = Sys.iswindows() ?
        "-dtr&rts&dtr&rts&rts&rts&rts&rts&rts&rts&rts&rts:-rts&-dtr" :
        "-dtr&rts,dtr,rts:-rts&-dtr,dtr"

    cmd = `$flasher -i $gpio_seq -w $hex_file -v -g 0x08000000 -b 115200 $puerto`
    run(cmd)

    println("\n✅ ¡PROCESO COMPLETADO EXITOSAMENTE!")
    println("   LQR calculado ➔ lqr_gain_mode<N>.h y lqr_mode.h escritos ➔ C compilado ➔ .hex generado ➔ STM32 Flasheada.")
end

# ------------------------------------------------------------------------------
# 4. FUNCIÓN RECEPTORA DEL CONTROLADOR
# ------------------------------------------------------------------------------
# Modo 0: set_ss_controller(K; mode=0, ref_pos=0.0, ref_yaw=0.0)
# Modo 1: set_ss_controller(K; mode=1, ref_vel=0.1, ref_yaw=deg2rad(0))
# Modo 2: set_ss_controller(K; mode=2, ref_pos=0.0, ref_wd=deg2rad(10))
# Todas las referencias se escriben siempre (lqr_mode.h): las que no use el modo
# programado quedan listas para cuando se cambie de modo con Set_Control_Mode().
function set_ss_controller(K_matrix; mode = 0, ref_pos = 0.0, ref_vel = 0.0, ref_yaw = 0.0, ref_wd = 0.0)
    write_controller(K_matrix; mode, ref_pos, ref_vel, ref_yaw, ref_wd)
    puerto = find_port()
    update_tasks_com_port(puerto)
    compile_and_flash(puerto)
end


function place2(A, B, polos )
    # --- Asignación por Submodos Simétricos ---
    A_long = A[1:4, 1:4]
    B_long = B[1:4, 1:1] + B[1:4, 2:2] # Ambas ruedas avanzan juntas
    polos_long = polos[1:4]

    K_long = place(A_long, B_long, polos_long)

    A_yaw = A[5:6, 5:6]
    B_yaw = B[5:6, 1:1] - B[5:6, 2:2] # Giro diferencial
    polos_yaw = polos[5:6]

    K_yaw = place(A_yaw, B_yaw, polos_yaw)

    # --- Reconstrucción de la Matriz K (2x6) ---
    K_matrix = [
        K_long K_yaw ;
        K_long -K_yaw
    ]

    # Aseguramos que K1, K2, K3, K4 tengan el mismo signo negativo que requiere C
    if K_matrix[1, 3] < 0 && K_matrix[1, 1] > 0
        K_matrix[1, 1:2] .= -K_matrix[1, 1:2]
        K_matrix[2, 1:2] .= -K_matrix[2, 1:2]
    end
    return K_matrix
end