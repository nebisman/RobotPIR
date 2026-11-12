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
# 2. GENERACIÓN DE C HEADER (lqr_gain.h)
# ------------------------------------------------------------------------------
function write_controller(K; ref_pos = 0.0, ref_vel=0.0, ref_yaw = 0.0, ref_wd=0.0,   type = 0, filepath=nothing)
    

    if isnothing(filepath)
        app_dir = joinpath(@__DIR__, "..", "APP")
        filepath = isdir(app_dir) ? joinpath(app_dir, "lqr_gain.h") : joinpath(@__DIR__, "lqr_gain.h")
    end

    # Se escribe primero en memoria: si las dimensiones de K no son válidas,
    # se lanza el error sin tocar el lqr_gain.h existente.
    io = IOBuffer()
    m, n = size(K)



    if  type == 0 # control de ganancia pura
        if (m != 2)  | (n != 6)
            error(" la matriz debe ser 2X6")
        end
            @printf(io, "// Ganancias LQR calculadas para los 6 estados del robot\n")
            @printf(io, "/* =================================================================== */\n")
            @printf(io, "/* ARCHIVO GENERADO AUTOMÁTICAMENTE DESDE JULIA                        */\n")
            @printf(io, "/* NO EDITAR MANUALMENTE - Ejecutar diseno_controlador.jl              */\n")
            @printf(io, "/* =================================================================== */\n")
            @printf(io, "#ifndef __LQR_GAIN_H\n")
            @printf(io, "#define __LQR_GAIN_H\n\n")
            @printf(io, "// Ganancias LQR calculadas para los 6 estados del robot\n")
            @printf(io, "#define K11_LQR    %.10f  // Posición X\n",         K[1,1])
            @printf(io, "#define K12_LQR    %.10f  // Velocidad X\n",        K[1,2])
            @printf(io, "#define K13_LQR    %.10f  // Ángulo Pitch\n",       K[1,3])
            @printf(io, "#define K14_LQR    %.10f  // Vel. Angular Pitch\n", K[1,4])
            @printf(io, "#define K15_LQR    %.10f  // Ángulo Yaw\n",         K[1,5])
            @printf(io, "#define K16_LQR    %.10f  // Vel. Angular Yaw\n",   K[1,6])
            @printf(io, "#define K21_LQR    %.10f  // Posición X\n",         K[2,1])
            @printf(io, "#define K22_LQR    %.10f  // Velocidad X\n",        K[2,2])
            @printf(io, "#define K23_LQR    %.10f  // Ángulo Pitch\n",       K[2,3])
            @printf(io, "#define K24_LQR    %.10f  // Vel. Angular Pitch\n", K[2,4])
            @printf(io, "#define K25_LQR    %.10f  // Ángulo Yaw\n",         K[2,5])
            @printf(io, "#define K26_LQR    %.10f  // Vel. Angular Yaw\n",   K[2,6])
            @printf(io, "#define Ki11_LQR     %.10f  // Constante integral x\n", 0.0)    
            @printf(io, "#define Ki12_LQR     %.10f  // Constante integral Yaw\n", 0.0)   
            @printf(io, "#define Ki21_LQR     %.10f  // Constante integral x\n", 0.0)    
            @printf(io, "#define Ki22_LQR     %.10f  // Constante integral Yaw\n", 0.0)   
            @printf(io, "#define REFPOS_LQR   %.10f  // referencia pos x \n", ref_pos)
            @printf(io, "#define REFPSI_LQR   %.10f  // referencia angulo Yaw \n", ref_yaw * pi/180.0)
            @printf(io, "#define REFVEL_LQR   %.10f  // referencia pos x \n", 0.0)
            @printf(io, "#define REFW_LQR     %.10f  // referencia angulo Yaw \n", 0.0)
            @printf(io, "#endif /* __LQR_GAIN_H */\n")

    elseif type == 1 # control con acción integral en la posición    
        if (m != 2)  | (n != 7)
            error(" la matriz debe ser 2 X 7")
        end
            @printf(io, "// Ganancias LQR calculadas para los 7 estados del robot\n")
            @printf(io, "/* =================================================================== */\n")
            @printf(io, "/* ARCHIVO GENERADO AUTOMÁTICAMENTE DESDE JULIA                        */\n")
            @printf(io, "/* NO EDITAR MANUALMENTE - Ejecutar diseno_controlador.jl              */\n")
            @printf(io, "/* =================================================================== */\n")
            @printf(io, "#ifndef __LQR_GAIN_H\n")
            @printf(io, "#define __LQR_GAIN_H\n\n")
            @printf(io, "// Ganancias LQR calculadas para los 6 estados del robot\n")
            @printf(io, "#define K11_LQR    %.10f  // Posición X\n",         K[1,1])
            @printf(io, "#define K12_LQR    %.10f  // Velocidad X\n",        K[1,2])
            @printf(io, "#define K13_LQR    %.10f  // Ángulo Pitch\n",       K[1,3])
            @printf(io, "#define K14_LQR    %.10f  // Vel. Angular Pitch\n", K[1,4])
            @printf(io, "#define K15_LQR    %.10f  // Ángulo Yaw\n",         K[1,5])
            @printf(io, "#define K16_LQR    %.10f  // Vel. Angular Yaw\n",   K[1,6])
            @printf(io, "#define K21_LQR    %.10f  // Posición X\n",         K[2,1])
            @printf(io, "#define K22_LQR    %.10f  // Velocidad X\n",        K[2,2])
            @printf(io, "#define K23_LQR    %.10f  // Ángulo Pitch\n",       K[2,3])
            @printf(io, "#define K24_LQR    %.10f  // Vel. Angular Pitch\n", K[2,4])
            @printf(io, "#define K25_LQR    %.10f  // Ángulo Yaw\n",         K[2,5])
            @printf(io, "#define K26_LQR    %.10f  // Vel. Angular Yaw\n",   K[2,6])
            @printf(io, "#define Ki11_LQR     %.10f  // Constante integral x\n",K[1,7])    
            @printf(io, "#define Ki12_LQR     %.10f  // Constante integral Yaw\n", 0.0)   
            @printf(io, "#define Ki21_LQR     %.10f  // Constante integral x\n", K[2,7])    
            @printf(io, "#define Ki22_LQR     %.10f  // Constante integral Yaw\n", 0.0)   
            @printf(io, "#define REFPOS_LQR     %.10f  // referencia pos x \n", ref_pos)
            @printf(io, "#define REFPSI_LQR     %.10f  // referencia angulo Yaw \n", ref_yaw * pi/180.0)
            @printf(io, "#define REFVEL_LQR   %.10f  // referencia pos x \n", 0.0)
            @printf(io, "#define REFW_LQR     %.10f  // referencia angulo Yaw \n", 0.0)
            @printf(io, "#endif /* __LQR_GAIN_H */\n")
     
    elseif type == 2 # control con acción integral en el yaw
             if (m != 2)  | (n != 7)
               error(" la matriz debe ser 2 X 7")
             end

            @printf(io, "// Ganancias LQR calculadas para los 7 estados del robot\n")
            @printf(io, "/* =================================================================== */\n")
            @printf(io, "/* ARCHIVO GENERADO AUTOMÁTICAMENTE DESDE JULIA                        */\n")
            @printf(io, "/* NO EDITAR MANUALMENTE - Ejecutar diseno_controlador.jl              */\n")
            @printf(io, "/* =================================================================== */\n")
            @printf(io, "#ifndef __LQR_GAIN_H\n")
            @printf(io, "#define __LQR_GAIN_H\n\n")
            @printf(io, "// Ganancias LQR calculadas para los 6 estados del robot\n")
            @printf(io, "#define K11_LQR    %.10f  // Posición X\n",         K[1,1])
            @printf(io, "#define K12_LQR    %.10f  // Velocidad X\n",        K[1,2])
            @printf(io, "#define K13_LQR    %.10f  // Ángulo Pitch\n",       K[1,3])
            @printf(io, "#define K14_LQR    %.10f  // Vel. Angular Pitch\n", K[1,4])
            @printf(io, "#define K15_LQR    %.10f  // Ángulo Yaw\n",         K[1,5])
            @printf(io, "#define K16_LQR    %.10f  // Vel. Angular Yaw\n",   K[1,6])
            @printf(io, "#define K21_LQR    %.10f  // Posición X\n",         K[2,1])
            @printf(io, "#define K22_LQR    %.10f  // Velocidad X\n",        K[2,2])
            @printf(io, "#define K23_LQR    %.10f  // Ángulo Pitch\n",       K[2,3])
            @printf(io, "#define K24_LQR    %.10f  // Vel. Angular Pitch\n", K[2,4])
            @printf(io, "#define K25_LQR    %.10f  // Ángulo Yaw\n",         K[2,5])
            @printf(io, "#define K26_LQR    %.10f  // Vel. Angular Yaw\n",   K[2,6])
            @printf(io, "#define Ki11_LQR     %.10f  // Constante integral x\n", 0.0)    
            @printf(io, "#define Ki12_LQR     %.10f  // Constante integral Yaw\n", K[1,7])   
            @printf(io, "#define Ki21_LQR     %.10f  // Constante integral x\n", 0.0)    
            @printf(io, "#define Ki22_LQR     %.10f  // Constante integral Yaw\n", K[2,7])   
            @printf(io, "#define REFPOS_LQR     %.10f  // referencia pos x \n", ref_pos)
            @printf(io, "#define REFPSI_LQR     %.10f  // referencia angulo Yaw \n", ref_yaw * pi/180.0)
            @printf(io, "#define REFVEL_LQR   %.10f  // referencia pos x \n", 0.0)
            @printf(io, "#define REFW_LQR     %.10f  // referencia angulo Yaw \n", 0.0)
            @printf(io, "#endif /* __LQR_GAIN_H */\n")

        elseif type == 3 # control con acción de velocidad
             if (m != 2)  | (n != 6)
               error(" la matriz debe ser 2 X 6")
             end

            @printf(io, "// Ganancias LQR calculadas para los 7 estados del robot\n")
            @printf(io, "/* =================================================================== */\n")
            @printf(io, "/* ARCHIVO GENERADO AUTOMÁTICAMENTE DESDE JULIA                        */\n")
            @printf(io, "/* NO EDITAR MANUALMENTE - Ejecutar diseno_controlador.jl              */\n")
            @printf(io, "/* =================================================================== */\n")
            @printf(io, "#ifndef __LQR_GAIN_H\n")
            @printf(io, "#define __LQR_GAIN_H\n\n")
            @printf(io, "// Ganancias LQR calculadas para los 6 estados del robot\n")
            @printf(io, "#define K11_LQR    %.10f  // Posición X\n",          0.0)
            @printf(io, "#define K12_LQR    %.10f  // Velocidad X\n",        K[1,1])
            @printf(io, "#define K13_LQR    %.10f  // Ángulo Pitch\n",       K[1,2])
            @printf(io, "#define K14_LQR    %.10f  // Vel. Angular Pitch\n", K[1,3])
            @printf(io, "#define K15_LQR    %.10f  // Ángulo Yaw\n",         K[1,4])
            @printf(io, "#define K16_LQR    %.10f  // Vel. Angular Yaw\n",   K[1,5])
            @printf(io, "#define K21_LQR    %.10f  // Posición X\n",          0.0 )
            @printf(io, "#define K22_LQR    %.10f  // Velocidad X\n",        K[2,1])
            @printf(io, "#define K23_LQR    %.10f  // Ángulo Pitch\n",       K[2,2])
            @printf(io, "#define K24_LQR    %.10f  // Vel. Angular Pitch\n", K[2,3])
            @printf(io, "#define K25_LQR    %.10f  // Ángulo Yaw\n",         K[2,4])
            @printf(io, "#define K26_LQR    %.10f  // Vel. Angular Yaw\n",   K[2,5])
            @printf(io, "#define Ki11_LQR     %.10f  // Constante integral v\n", K[1,6])    
            @printf(io, "#define Ki12_LQR     %.10f  // Constante integral w_delta\n", 0.0)   
            @printf(io, "#define Ki21_LQR     %.10f  // Constante integral v\n", K[2,6])    
            @printf(io, "#define Ki22_LQR     %.10f  // Constante integral w_delta\n", 0.0)   
            @printf(io, "#define REFPOS_LQR     %.10f  // referencia velocidad  \n", 0.0)
            @printf(io, "#define REFPSI_LQR     %.10f  // referencia wdelta \n", 0.0)
            @printf(io, "#define REFVEL_LQR   %.10f  // referencia pos x \n", ref_vel)
            @printf(io, "#define REFW_LQR     %.10f  // referencia angulo Yaw \n", ref_wd)
            @printf(io, "#endif /* __LQR_GAIN_H */\n")
    
    else
        error("modo no valido")
    end


    write(filepath, take!(io))

    println("✅ Archivo 'lqr_gain.h' generado correctamente en:\n   Ruta: $filepath")
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
    println("   LQR calculado ➔ lqr_gain.h escrito ➔ C compilado ➔ .hex generado ➔ STM32 Flasheada.")
end

# ------------------------------------------------------------------------------
# 4. FUNCIÓN RECEPTORA DEL CONTROLADOR
# ------------------------------------------------------------------------------
function set_ss_controller(K_matrix; ref_pos = 0.0, ref_vel=0.0, ref_yaw = 0.0, ref_wd=0.0,   type = 0,)
    write_controller(K_matrix; ref_pos, ref_vel, ref_yaw, ref_wd,  type)
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