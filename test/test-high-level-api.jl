
using LibSerialPort


function test_nonblocking_serial_loopback(sp::SerialPort)

    println("\n[TEST] Read any incoming data for ~1 second...")
    for i = 1:1000
        print(read(sp, String))
        sleep(0.001)
    end
    println()

    flush(sp, buffer=SP_BUF_BOTH)

    print("\n\n[TEST] Serial loopback - ")
    println("Send 100 short messages and read whatever comes back...")

    for i = 1:100
        write(sp, "Test message $i\n")
        print(read(sp, String))
        i == 100 && write(sp, "done")
    end

    # Allow up to 100 more ms to get the remaining data
    for i = 1:100
        data = read(sp, String)
        print(data)
        if occursin(data, "done")
            break
        end
        sleep(0.001)
    end

    println()

    flush(sp, buffer=SP_BUF_BOTH)
end

function test_readline(sp::SerialPort)

    println("\n[TEST] Read any incoming data for ~1 second...")
    for i = 1:1000
        print(read(sp, String))
        sleep(0.001)
    end
    println()

    flush(sp, buffer=SP_BUF_BOTH)

    print("\n\n[TEST] Serial loopback - ")
    println("Send 100 short messages and read whatever comes back...")

    for i = 1:100
        write(sp, "Test message $i\n")
        sleep(0.001)
        received_message = readuntil(sp, '\n') # same as readline(sp)
        println(chomp(received_message))

        # Trigger an EOF when done writing so readuntil doesn't hang forever
        if i == 100
            seteof(sp, true)
        end
    end
    reseteof(sp)

    flush(sp, buffer=SP_BUF_BOTH)
end

function test_high_level_api(args...)

    if length(args) != 2
        println("Usage: $(basename(@__FILE__)) port baudrate")
        println("Available ports:")
        list_ports()
        return
    end

    sp = open(args[1], parse(Int, args[2]))

    print_port_metadata(sp)
    print_port_settings(sp)

    test_nonblocking_serial_loopback(sp)
    test_readline(sp)

    close(sp)
end

test_high_level_api(ARGS...)
