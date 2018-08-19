# Basic serial console.
# Data is read from a serial device and lines (but not individual keypresses)
# are written to the device asynchronously.

using LibSerialPort

function serial_loop(sp::SerialPort)
    input_line = ""
    mcu_message = ""

    println("Starting I/O loop. Press ESC [return] to quit")

    while true
        # Poll for new data without blocking
        @async input_line = readline(keep=true)
        @async mcu_message *= readstring(sp)

        # Alternative read method:
        # Requires setting a timeout and may cause bottlenecks
        # @async mcu_message = readuntil(sp, "\r\n", 50)

        occursin("\e", input_line) && exit()

        # Send user input to device
        if endswith(input_line, '\n')
            write(sp, "$input_line")
            input_line = ""
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