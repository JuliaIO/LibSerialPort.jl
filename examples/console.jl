# Basic serial console.
# Data is read from a serial device and lines (but not individual keypresses)
# are written to the device asynchronously.

using LibSerialPort

function serial_loop(sp::SerialPort)
    user_input = ""
    mcu_message = ""

    println("Starting I/O loop. Press ESC [return] to quit")
    sleep(3)

    while true
        # Poll for new data without blocking
        @async user_input = readline(keep=true)
        @async mcu_message *= String(nonblocking_read(sp))

        occursin("\e", user_input) && exit()

        # Send user input to device
        if endswith(user_input, '\n')
            write(sp, "$user_input")
            user_input = ""
        end

        # Print message from device
        if occursin("\r\n", mcu_message)
            lines = split(mcu_message, "\r\n")
            while length(lines) > 1
                println(popfirst!(lines))
            end
            mcu_message = lines[1]
        end

        # Give the queued tasks a chance to run
        sleep(0.0001)
    end
end

function console(args...)

    if length(args) != 2
        println("Usage: $(basename(@__FILE__)) port baudrate")
        println("Available ports:")
        list_ports()
        return
    end

    # Open a serial connection to the microcontroller
    mcu = open(args[1], parse(Int, args[2]))

    serial_loop(mcu)
end

console(ARGS...)