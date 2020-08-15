"""
This test is written to run on a machine connected to an Arduino-compatible
microcontroller running examples/serial_example.ino.

In bash, `export LIBSERIALPORT_DEBUG=` to display debug info (`unset` to clear)
"""

using LibSerialPort
using Test


"""
Use all wrapped functions to display version info (tested on 0.1.1)
"""
function print_version()
    println(sp_get_major_package_version())
    println(sp_get_minor_package_version())
    println(sp_get_micro_package_version())
    println(sp_get_package_version_string())
    println(sp_get_current_lib_version())
    println(sp_get_revision_lib_version())
    println(sp_get_age_lib_version())
    println(sp_get_lib_version_string())
end

"""
Serial loopback test using blocking read/write.
A microcontroller is connected over USB/USART and is echoing a test message
written here.
"""
function test_blocking_serial_loopback(port::LibSerialPort.Port,
    write_timeout_ms::Integer, read_timeout_ms::Integer)

    println("\nTesting serial loopback with blocking write/read functions...")

    # Clear serial buffers (deletes any buffered data)
    sp_flush(port, SP_BUF_BOTH)
    sleep(0.5)

    function loopback_with_reallocation()
        """Use sp_blocking_read, which allocates a new read buffer internally on every call.
        """
        for i = 1:10
            message = collect(UInt8(0):UInt8(255))
            sp_blocking_write(port, message, write_timeout_ms)

            sleep(0.1)

            num_bytes_to_read = Int(sp_input_waiting(port))
            if num_bytes_to_read > 0

                nbytes_read, bytes_read = sp_blocking_read(port, num_bytes_to_read, read_timeout_ms)

                println("($i) $nbytes_read, $num_bytes_to_read")

                @test nbytes_read == num_bytes_to_read

                if bytes_read != message
                    @warn("Received data does not match transmitted data:")
                    println("Bytes received: ", bytes_read)
                    println("Bytes missing:  ", setdiff(message, bytes_read));
                    println("Bytes added:    ", setdiff(bytes_read, message));
                end

                @test bytes_read == message
            end
        end
    end

    function loopback_with_preallocation()
        """Use sp_blocking_read with a preallocated read buffer.
        """
        for i = 1:10
            message = "Test message $i\n"
            sp_blocking_write(port, message, write_timeout_ms)

            sleep(0.1)

            num_bytes_to_read = Int(sp_input_waiting(port))
            if num_bytes_to_read > 0

                nbytes_read = 0
                bytes_read = zeros(UInt8, 1024)

                for j in 1:num_bytes_to_read
                    b = Ref{UInt8}(0)
                    nbytes_read += Int(sp_blocking_read(port, b, 1, read_timeout_ms))
                    bytes_read[j] = b.x
                end

                response = String(bytes_read[1:nbytes_read])

                @test nbytes_read == num_bytes_to_read

                println("($i) $nbytes_read, $num_bytes_to_read\n$response")
            end
        end
    end

    @time loopback_with_reallocation()
    @time loopback_with_preallocation()
end

function test_nonblocking_serial_loopback(port::LibSerialPort.Port)

    sp_flush(port, SP_BUF_BOTH)

    println("\nTesting serial loopback with nonblocking write/read functions...")

    function loops()
        for i = 1:10
            sp_nonblocking_write(port, "Test message $i\n")
        end

        sleep(0.1)  # Give the connected device time to echo back

        for i = 1:10
            num_bytes_to_read = Int(sp_input_waiting(port))
            if num_bytes_to_read > 0
                nbytes_read, bytes = sp_nonblocking_read(port, num_bytes_to_read)
                @test nbytes_read == num_bytes_to_read
                sleep(0.1)
            end
        end
    end

    @time loops()
end

"""
This example demonstrates serial communication with one port. The default
configuration is 115200-8-N-1, i.e. 115200 bps with 8 data bits, no parity check,
and one stop bit. The baud rate is overridden on the command line with a
second argument. Hardware and software flow control measures are disabled.
"""
function test_low_level_api(args...)

    nargs = length(args)
    if nargs == 0
        println("Usage: $(basename(@__FILE__)) port [baudrate]")
        println("Available ports:")
        list_ports()
        return
    end

    if !ispath(args[1])
        println("Not found: ", args[1])
        return
    end

    port = sp_get_port_by_name(args[1]) # e.g. "/dev/cu.wchusbserial1410"
    baudrate = nargs >= 2 ? parse(Int, args[2]) : 115200

    print_version()
    list_ports()

    sp_open(port, SP_MODE_READ_WRITE)

    sp_set_baudrate(port, baudrate)
    sp_set_bits(port, 8)
    sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE)

    sleep(2)

    @testset "Blocking read/write" begin
        test_blocking_serial_loopback(port, 10, 10)
    end

    @testset "Nonblocking read/write" begin
        test_nonblocking_serial_loopback(port)
    end

    sp_close(port)
    sp_free_port(port)
end

test_low_level_api(ARGS...)
