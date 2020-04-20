
using LibSerialPort
using Test

function test_blocking_serial_loopback(sp::SerialPort)

    # This "tests" `read` indirectly, but has an important side effect of flushing
    # the input buffer, putting the test into a more well-defined state.
    println("\nRead all data currently waiting in the serial input buffer...")
    println(String(read(sp)))

    # Read 10 lines, blocking on each line for sp.read_timeout_ms.
    # The default read timeout is 0, which blocks indefinitely.
    for i = 1:10
        println(i, " ", readline(sp))
    end
    println()

    num_messages = 10

    for i = 1:num_messages
        msg = "test message $i\n"
        print("Wrote ", msg)
        write(sp, msg)
    end

    # Check for "Received test message _" responses.
    lines = [readline(sp) for _ in 1:num_messages+5]
    num_responses = count(line->startswith(line, "Received test message"), lines)

    [println(line) for line in lines]

    @test num_responses == num_messages
end


function test_nonblocking_serial_loopback(sp::SerialPort)

    println("\nnonblocking_read...")
    for i = 1:100
        print(String(nonblocking_read(sp)))
        sleep(0.001)
    end
    println()

    write(sp, "test message.\n")

    sleep(0.1)

    result = String(nonblocking_read(sp))
    show(result)
    println()
    @test occursin("test message", result)
end


function test_readline(sp::SerialPort)

    println("\n[TEST] Read any incoming data for ~1 second...")
    for i = 1:1000
        print(read(sp, String))
        sleep(0.001)
    end
    println()

    sp_flush(sp, SP_BUF_BOTH)

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

    sp_flush(sp, SP_BUF_BOTH)
end

function test_high_level_api(args...)

    if length(args) != 2
        println("Usage: $(basename(@__FILE__)) port baudrate")
        println("Available ports:")
        list_ports()
        return
    end

    if !ispath(args[1])
        println("Not found: ", args[1])
        return
    end

    sp = open(args[1], parse(Int, args[2]))

    print_port_metadata(sp)
    print_port_settings(sp)

    @testset "Blocking read/write" begin
        test_blocking_serial_loopback(sp)
    end

    @testset "Nonblocking read" begin
        test_nonblocking_serial_loopback(sp)
    end

    # test_readline(sp)

    close(sp)
    return
end

test_high_level_api(ARGS...)
