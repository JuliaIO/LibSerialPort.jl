# `export LIBSERIALPORT_DEBUG=` to display debug info (`unset` to clear)

include("libserialport.jl")

"""
Print libserialport version (tested on 0.1.1)
"""
function print_library_version()
    # TODO replace this with wrapped version
    ver = ccall((:sp_get_package_version_string, "libserialport"), Ptr{UInt8}, ())
    println(bytestring(ver))
end

"""
Print a list of currently visible ports, along with some basic info
"""
function list_ports()
    ports = sp_list_ports()
    nports_overestimate = 32

    for port in pointer_to_array(ports, nports_overestimate, false)
        port == C_NULL && return

        println(sp_get_port_name(port))
        println("\tDescription:\t",    sp_get_port_description(port))
        println("\tTransport type:\t", sp_get_port_transport(port))
    end

    sp_free_port_list(ports)
    return
end

function print_port_info(port)
    println("Name:\t",              sp_get_port_name(port))
    println("Manufacturer:\t",      sp_get_port_usb_manufacturer(port))
    println("Product:\t",           sp_get_port_usb_product(port))
    println("USB serial number:\t", sp_get_port_usb_serial(port))
    println("Bluetooth address:\t", sp_get_port_bluetooth_address(port))
    println("File descriptor:\t",   sp_get_port_handle(port))

    bus, addr = sp_get_port_usb_bus_address(port)
    if bus != -1
        println("USB bus #:\t",   bus)
        println("Address on bus:\t",  addr)
    end

    vid, pid = sp_get_port_usb_vid_pid(port)
    if vid != -1
        println("Vendor ID:\t",   vid)
        println("Product ID:\t",  pid)
    end
end

"""
Print settings currently stored in sp_port_config struct
"""
function print_config_info(config)
    println("\tbaudrate\t", sp_get_config_baudrate(config))
    println("\tbits\t",     sp_get_config_bits(config))
    println("\tparity\t",   sp_get_config_parity(config))
    println("\tstopbits\t", sp_get_config_stopbits(config))
    println("\tRTS\t",      sp_get_config_rts(config))
    println("\tCTS\t",      sp_get_config_cts(config))
    println("\tDTR\t",      sp_get_config_dtr(config))
    println("\tDSR\t",      sp_get_config_dsr(config))
    println("\tXonXoff\t",  sp_get_config_xon_xoff(config))
end

function main()
    print_library_version()
    list_ports()

    # port = sp_get_port_by_name("/dev/cu.usbserial-00002014")
    port = sp_get_port_by_name("/dev/cu.wchusbserial1410")

    sp_open(port, SP_MODE_READ_WRITE)

    print_port_info(port)

    # config = sp_new_config()
    config = sp_get_config(port)

    println("\nDefault config settings:")
    print_config_info(config)

    println("\nUpdating config settings:")
    sp_set_config_baudrate(config, 57600)
    sp_set_config_bits(config, 8)
    sp_set_config_parity(config, SP_PARITY_NONE)
    sp_set_config_stopbits(config, 1)

    # sp_get_port_handle(port)
    # sp_set_bits(port, 8)
    # sp_set_baudrate(port, 115200)
    # sp_set_parity(port, SP_PARITY_NONE)

    println("\nUpdated config settings:")
    print_config_info(config)

    println("\nAssigning to port...")
    println(sp_set_config(port, config))

    println("\nSettings for port...")
    config = sp_get_config(port)
    print_config_info(config)

    println("\nClosing and freeing port. Over and out.")
    sp_close(port)
    sp_free_port(port)
end

main()

# println(@__FILE__, ", ", @__LINE__)

# *****
