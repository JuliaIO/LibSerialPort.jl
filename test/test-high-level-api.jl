
using LibSerialPort

function test_nonblocking_serial_loopback(sp::SerialPort)

    println("\n[TEST] Read any incoming data for ~1 second...")
    for i = 1:1000
        print(readstring(sp))
        sleep(0.001)
    end
    println()

    flush(sp, buffer=SP_BUF_BOTH)

    print("\n\n[TEST] Serial loopback - ")
    println("Send 100 short messages and read whatever comes back...")

    tic()
    for i = 1:100
        write(sp, "Test message $i\n")
        print(readstring(sp))
        i == 100 && write(sp, "done")
    end

    # Allow up to 100 more ms to get the remaining data
    for i = 1:100
        data = readstring(sp)
        print(data)
        if occursin(data, "done")
            break
        end
        sleep(0.001)
    end

    println()
    toc()

    flush(sp, buffer=SP_BUF_BOTH)
end

function test_readline(sp::SerialPort)

    println("\n[TEST] Read any incoming data for ~1 second...")
    for i = 1:1000
        print(readstring(sp))
        sleep(0.001)
    end
    println()

    flush(sp, buffer=SP_BUF_BOTH)

    print("\n\n[TEST] Serial loopback - ")
    println("Send 100 short messages and read whatever comes back...")

    tic()
    for i = 1:100
        write(sp, "Test message $i\n")
        received_message = readuntil(sp, '\n') # same as readline(sp)
        print(received_message)

        # Trigger an EOF when done writing so readuntil doesn't hang forever
        if i == 100
            seteof(sp, true)
        end
    end
    reseteof(sp)
    toc()

    flush(sp, buffer=SP_BUF_BOTH)
end

function main()

    if length(ARGS) != 2
        println("Usage: $(basename(@__FILE__)) port baudrate")
        println("Available ports:")
        list_ports()
        return
    end

    sp = open(ARGS[1], parse(Int, ARGS[2]))

    print_port_metadata(sp)
    print_port_settings(sp)

    test_nonblocking_serial_loopback(sp)
    test_readline(sp)

    close(sp, delete=true)
end

main()
