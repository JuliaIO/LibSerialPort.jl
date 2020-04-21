
using LibSerialPort
using Test

function test_blocking_serial_loopback(sp::SerialPort)

    # This "tests" `read` indirectly, but has an important side effect of flushing
    # the input buffer, putting the test into a more well-defined state.
    println("\nRead all data currently waiting in the serial input buffer...")
    if bytesavailable(sp) > 0
        println(String(read(sp)))
    end

    for i = 1:10
        msg = "test message $i\n"
        write(sp, msg)
        print("Wrote ", msg)
        sleep(0.1)
        line = readline(sp)
        println(line)
        @test startswith(line, "Received test message")
    end

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

    sp.read_timeout_ms = 1000

    print_port_metadata(sp)
    print_port_settings(sp)

    sleep(2)  # MCU serial startup

    @testset "Blocking read/write" begin
        test_blocking_serial_loopback(sp)
    end

    @testset "Nonblocking read" begin
        test_nonblocking_serial_loopback(sp)
    end

    close(sp)
    return
end

test_high_level_api(ARGS...)
