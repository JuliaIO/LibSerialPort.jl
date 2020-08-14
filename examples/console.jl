"""
Basic line-buffering serial console.
Data is read  asynchronously from a serial device and user input is written
by line (not keypress).

If a serial device is connected that echoes bytes, each line of user input
should be duplicated.
"""

using LibSerialPort

function serial_loop(sp::SerialPort)
    user_input = ""
    mcu_message = ""

    println("Starting I/O loop. Press ESC [return] to quit")

    while true
        # Poll for new data without blocking
        @async user_input = readline(keep=true)
        @async mcu_message *= String(nonblocking_read(sp))

        occursin("\e", user_input) && exit()  # escape

        # Send user input to device with ENTER
        if endswith(user_input, '\n')
            write(sp, "$user_input")
            user_input = ""
        end

        # Print response from device as a line
        if occursin("\n", mcu_message)
            lines = split(mcu_message, "\n")
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
