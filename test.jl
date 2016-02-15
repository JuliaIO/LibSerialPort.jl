# `export LIBSERIALPORT_DEBUG=` to display debug info (`unset` to clear)

include("libserialport.jl")

function print_config_info(config)
    println("\tbaudrate\t", sp_get_config_baudrate(config))
    println("\tbits\t", sp_get_config_bits(config))
    println("\tparity\t", sp_get_config_parity(config))
    println("\tstopbits\t", sp_get_config_stopbits(config))
    println("\tRTS\t", sp_get_config_rts(config))
end

# Print libserialport version (tested on 0.1.1)
ver = ccall((:sp_get_package_version_string, "libserialport"), Ptr{UInt8}, ())
println(bytestring(ver))

# Print a list of discovered ports and (by default) some info about them
sp_list_ports()

# port = sp_get_port_by_name("/dev/cu.usbserial-00002014")
port = sp_get_port_by_name("/dev/cu.wchusbserial1410")
name = sp_get_port_name(port)

sp_open(port, SP_MODE_READ_WRITE)

println("name: ", name)

# config = sp_new_config()
config = sp_get_config(port)

println("Default config settings:")
print_config_info(config)

println("Updating config settings:")
sp_set_config_baudrate(config, 57600)
sp_set_config_bits(config, 8)
sp_set_config_parity(config, SP_PARITY_NONE)
sp_set_config_stopbits(config, 1)

# sp_get_port_handle(port)
# sp_set_bits(port, 8)
# sp_set_baudrate(port, 115200)
# sp_set_parity(port, SP_PARITY_NONE)

println("Updated config settings:")
print_config_info(config)

println("Assigning to port...")
println(sp_set_config(port, config))

println("Settings for port...")
config = sp_get_config(port)
print_config_info(config)

println("Closing and freeing port. Bye")
sp_close(port)
sp_free_port(port)

# println(@__FILE__, ", ", @__LINE__)

# *****
