# `export LIBSERIALPORT_DEBUG=` to display debug info (`unset` to clear)

using LibSerialPort

"""
Test that we can change some port configuration settings on a copy of the
provided port. Use two approaches to cover the various set and get functions.
The original port configuration should not be modified by these tests!
"""
function test_port_configuration(port::LibSerialPort.Port)
    # 1. (Direct) use setter functions for the port struct
    test_change_port_copy_method1(port)
    # 2. (Roundabout) get a new sp_port_configuration instance, modify it, then
    # copy its data fields to the port struct.
    test_change_port_copy_method2(port)
end

function test_change_port_copy_method1(port::LibSerialPort.Port)
    port2 = sp_copy_port(port)
    sp_close(port)
    sp_open(port2, SP_MODE_READ_WRITE)

    print("\n[TEST1] INITIAL ")
    print_port_settings(port2)
    println("[TEST1] changing port configuration settings.")
    sp_set_baudrate(port2, 115200)
    sp_set_bits(port2, 6)
    sp_set_parity(port2, SP_PARITY_EVEN)
    sp_set_stopbits(port2, 2)

    # Request to send / clear to send go as a pair.
    # They must be enabled or disabled together.
    sp_set_rts(port2, SP_RTS_OFF)
    sp_set_cts(port2, SP_CTS_IGNORE)

    sp_set_dtr(port2, SP_DTR_OFF)
    sp_set_dsr(port2, SP_DSR_IGNORE)

    sp_set_xon_xoff(port2, SP_XONXOFF_INOUT)

    print("[TEST1] UPDATED ")
    print_port_settings(port2)

    println("[TEST1] closing and deleting copied port")
    sp_close(port2)
    sp_free_port(port2)

    println("[TEST1] reopening original port")
    sp_open(port, SP_MODE_READ_WRITE)

    print("[TEST1] ORIGINAL ")
    print_port_settings(port)
end

function test_change_port_copy_method2(port::LibSerialPort.Port)
    port2 = sp_copy_port(port)
    sp_close(port)
    sp_open(port2, SP_MODE_READ_WRITE)

    # Either
    # config2 = sp_new_config()
    # Or
    config2 = sp_get_config(port2)

    print("\n[TEST2] INITIAL ")
    print_port_settings(config2)

    println("[TEST2] changing configuration settings.")
    sp_set_config_baudrate(config2, 115200)
    sp_set_config_bits(config2, 6)
    sp_set_config_parity(config2, SP_PARITY_EVEN)
    sp_set_config_stopbits(config2, 2)
    sp_set_config_rts(config2, SP_RTS_OFF)
    sp_set_config_cts(config2, SP_CTS_IGNORE)
    sp_set_config_dtr(config2, SP_DTR_OFF)
    sp_set_config_dsr(config2, SP_DSR_IGNORE)
    sp_set_config_xon_xoff(config2, SP_XONXOFF_INOUT)

    sp_set_config(port2, config2)
    sp_free_config(config2)

    print("[TEST2] UPDATED ")
    print_port_settings(port2)

    println("[TEST2] closing and deleting copied port")
    sp_close(port2)
    sp_free_port(port2)

    println("[TEST2] reopening original port")
    sp_open(port, SP_MODE_READ_WRITE)

    print("[TEST2] ORIGINAL ")
    print_port_settings(port)
end

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
    println()
    data = unsafe_string(sp_blocking_read(port, 1024, 2000))
    println(data)
    sp_drain(port)
    sp_flush(port, SP_BUF_BOTH)

    tic()
    for i = 1:100
        sp_blocking_write(port, Array{UInt8}("Test message $i\n"), write_timeout_ms)
        sp_drain(port)
        sp_flush(port, SP_BUF_OUTPUT)
        data = unsafe_string(sp_blocking_read(port, 128, read_timeout_ms))
        print(data)
        sp_flush(port, SP_BUF_INPUT)
    end
    toc()
end

function test_nonblocking_serial_loopback(port::LibSerialPort.Port)

    println("\n[TEST] Read any incoming data for ~1 second...")
    for i = 1:1000
        print(unsafe_string(sp_nonblocking_read(port, 1024)))
        sleep(0.001)
    end

    sp_flush(port, SP_BUF_BOTH)

    print("\n\n[TEST] Serial loopback - ")
    println("Send 100 short messages and read whatever comes back...")
    tic()
    for i = 1:100
        sp_nonblocking_write(port, Array{UInt8}("Test message $i\n"))
        sp_drain(port)
        print(unsafe_string(sp_nonblocking_read(port, 256)))
    end

    # Read and print any remaining data for ~50 ms
    for i = 1:50
        print(unsafe_string(sp_nonblocking_read(port, 256)))
        sleep(0.001)
    end

    sp_flush(port, SP_BUF_BOTH)
    toc()
end

"""
This example demonstrates serial communication with one port. The default
configuration is 9600-8-N-1, i.e. 9600 bps with 8 data bits, no parity check,
and one stop bit. The baud rate is overridden on the command line with a
second argument. Hardware and software flow control measures are disabled by
default.
"""
function main()

    nargs = length(ARGS)
    if nargs == 0
        println("Usage: test.jl port [baudrate]")
        println("Available ports:")
        list_ports()
        return
    end

    port = sp_get_port_by_name(ARGS[1]) # e.g. "/dev/cu.wchusbserial1410"
    baudrate = nargs >= 2 ? parse(Int, ARGS[2]) : 9600

    print_version()
    list_ports()

    sp_open(port, SP_MODE_READ_WRITE)

    test_port_configuration(port)

    sp_set_baudrate(port, baudrate)

    test_blocking_serial_loopback(port, 10, 40)

    test_nonblocking_serial_loopback(port)

    println("\nClosing and freeing port. Over and out!")
    sp_close(port)
    sp_free_port(port)
end

main()
