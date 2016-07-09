
type SerialPort <: IO
    ref::Port
    eof::Bool
end

"""
Constructor
"""
SerialPort(portname::AbstractString) = SerialPort(sp_get_port_by_name(portname), false)

"""
Set connection speed in bits per second. The library will return an error if bps
is not a valid/supported value.
"""
set_speed(sp::SerialPort, bps::Integer) = sp_set_baudrate(sp.ref, bps)

"""
Configure packet framing. Defaults to the most common "8N1" scheme.
https://en.wikipedia.org/wiki/Universal_asynchronous_receiver/transmitter#Data_framing
"""
function set_frame(sp::SerialPort;
    ndatabits::Integer=8,
    parity::SPParity=SP_PARITY_NONE,
    nstopbits::Integer=1)

    sp_set_bits(sp.ref, ndatabits)
    sp_set_parity(sp.ref, parity)
    sp_set_stopbits(sp.ref, nstopbits)
end

"""
Configure flow control settings. Many systems don't support all options.
If an unsupported option is requested, the library will return SP_ERR_SUPP.
"""
function set_flow_control(sp::SerialPort;
    rts::SPrts=SP_RTS_OFF,
    cts::SPcts=SP_CTS_IGNORE,
    dtr::SPdtr=SP_DTR_OFF,
    dsr::SPdsr=SP_DSR_IGNORE,
    xonxoff::SPXonXoff=SP_XONXOFF_DISABLED)

    # CTS and RTS must be enabled or disabled as a pair
    sp_set_rts(sp.ref, rts)
    sp_set_cts(sp.ref, cts)

    sp_set_dtr(sp.ref, dtr)
    sp_set_dsr(sp.ref, dsr)

    sp_set_xon_xoff(sp.ref, xonxoff)
end

"""
Print a list of currently visible ports, along with some basic info
"""
function list_ports(;nports_guess=64)
    ports = sp_list_ports()

    for port in pointer_to_array(ports, nports_guess, false)
        port == C_NULL && return

        println(sp_get_port_name(port))
        println("\tDescription:\t",    sp_get_port_description(port))
        println("\tTransport type:\t", sp_get_port_transport(port))
    end

    sp_free_port_list(ports)
    return
end


"""
Print info found for this port.
Note: port should be open to obtain a valid FD/handle before accessing fields.
"""
function print_port_metadata(sp::SerialPort; show_config::Bool=true)
    print_port_metadata(sp.ref, show_config=show_config)
end

function print_port_metadata(port::LibSerialPort.Port; show_config::Bool=true)
    println("\nPort name:\t",       sp_get_port_name(port))
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
    if show_config
        print_port_settings(port)
    end
end

"""
Print settings currently stored in sp_port_config struct
"""
function print_port_settings(config::LibSerialPort.Config)
    println("\tbaudrate\t", sp_get_config_baudrate(config))
    println("\tbits\t",     sp_get_config_bits(config))
    println("\tparity\t",   sp_get_config_parity(config))
    println("\tstopbits\t", sp_get_config_stopbits(config))
    println("\tRTS\t",      sp_get_config_rts(config))
    println("\tCTS\t",      sp_get_config_cts(config))
    println("\tDTR\t",      sp_get_config_dtr(config))
    println("\tDSR\t",      sp_get_config_dsr(config))
    println("\tXonXoff\t",  sp_get_config_xon_xoff(config))
    println("")
end

function print_port_settings(port::LibSerialPort.Port)
    println("Configuration for ", sp_get_port_name(port), ":")
    config = sp_get_config(port)
    print_port_settings(config)
    sp_free_config(config)
end

print_port_settings(sp::SerialPort) = print_port_settings(sp.ref)

function Base.open(sp::SerialPort; mode::SPMode=SP_MODE_READ_WRITE)
    sp_open(sp.ref, mode)
    return sp
end

"""
Convenience method with keyword args for common settings
"""
function Base.open(portname::AbstractString,
                   bps::Integer;
                   mode::SPMode=SP_MODE_READ_WRITE,
                   ndatabits::Integer=8,
                   parity::SPParity=SP_PARITY_NONE,
                   nstopbits::Integer=1)
    sp = SerialPort(sp_get_port_by_name(portname), false)
    sp_open(sp.ref, mode)
    set_speed(sp, bps)
    set_frame(sp, ndatabits=ndatabits, parity=parity, nstopbits=nstopbits)
    return sp
end

"""
Create and configure a SerialPort object.
Example: `open_serial_port("/dev/ttyACM0", 115200)`
"""
function open_serial_port(port_address::AbstractString, speed::Integer)
    sp = SerialPort(port_address)
    open(sp)
    set_speed(sp, speed)
    set_frame(sp, ndatabits=8, parity=SP_PARITY_NONE, nstopbits=1)
    return sp
end

function Base.close(sp::SerialPort; delete::Bool=false)

    # Flush first, as is done in other close() methods in Base
    sp_flush(sp.ref, SP_BUF_BOTH)

    sp_close(sp.ref)

    if delete
        sp_free_port(sp.ref)
    end
    return sp
end

function Base.flush(sp::SerialPort; buffer::SPBuffer=SP_BUF_BOTH)
    sp_flush(sp.ref, buffer)
end

function Base.write(sp::SerialPort, data::String)
    sp_nonblocking_write(sp.ref, Array{UInt8}(data))
    sp_drain(sp.ref)
end

function Base.write(sp::SerialPort, data::UInt8)
    sp_nonblocking_write(sp.ref, [data])
    sp_drain(sp.ref)
end

Base.write(sp::SerialPort, i::Int64) = Base.write(sp, "$i")

# TODO user-controlled precision. How to do string interpolation in macros?
Base.write(sp::SerialPort, f::Float32) = Base.write(sp, @sprintf("%.3f", f))
Base.write(sp::SerialPort, f::Float64) = Base.write(sp, @sprintf("%.3f", f))

Base.eof(sp::SerialPort) = sp.eof == true

seteof(sp::SerialPort, state::Bool) = sp.eof = state
reseteof(sp::SerialPort) = seteof(sp, false)

function Base.read(sp::SerialPort, ::Type{UInt8})
    byte_array = sp_nonblocking_read(sp.ref, 1)
    return (length(byte_array) > 0) ? UInt8(byte_array[1]) : 0x00
end

function Base.read(sp::SerialPort, ::Type{Char})
    byte_array = sp_nonblocking_read(sp.ref, 1)
    return (length(byte_array) > 0) ? Char(byte_array[1]) : '\0'
end

"""
Read until the specified delimiting byte (e.g. '\n') is encountered, or until
timeout_ms has elapsed, whichever comes first.
"""
function Base.readuntil(sp::SerialPort, delim::Char, timeout_ms::Integer)
    # TODO: this is in Base (io.jl) - is it also needed here?
    # if delim < Char(0x80)
    #     return String(readuntil(sp, delim % UInt8))
    # end

    start_time = time_ns()
    out = IOBuffer()
    while !eof(sp)
        if (time_ns() - start_time)/1e6 > timeout_ms
            break
        end
        if nb_available(sp) > 0
            c = read(sp, Char)
            write(out, c)
            if c == delim
                break
            end
        end
    end
    return takebuf_string(out)
end

"""
Read until the specified delimiting byte (e.g. '\n') is encountered, or until
timeout_ms has elapsed, whichever comes first.
"""
function readuntil(sp::SerialPort, delim::Char, timeout_ms::Integer)
    # TODO: this is in Base (io.jl) - is it also needed here?
    # if delim < Char(0x80)
    #     return String(readuntil(sp, delim % UInt8))
    # end

    start_time = time_ns()
    out = IOBuffer()
    while !eof(sp)
        if (time_ns() - start_time)/1e6 > timeout_ms
            break
        end
        if nb_available(sp) > 0
            c = read(sp, Char)
            write(out, c)
            if c == delim
                break
            end
        end
    end
    return takebuf_string(out)
end

# function Base.readuntil(sp::SerialPort, delim::Char)
#     out = Char[]
#     while !eof(sp)
#         if nb_available(sp) > 0
#             c = read(sp, Char)
#             push!(out, c)
#             if c == delim
#                 break
#             end
#         end
#     end
#     return join(out)
# end

Base.nb_available(sp::SerialPort) = Int(sp_input_waiting(sp.ref))

function Base.readbytes(sp::SerialPort, nbytes::Integer)
    sp_nonblocking_read(sp.ref, nbytes)
end

"""
Read everything in libserialport's input buffer, one byte at a time, until it
is empty. Returns an ASCIIString.
"""
function Base.readall(sp::SerialPort)
    result = Char[]
    while Int(nb_available(sp)) > 0
        byte = Base.readbytes(sp, 1)[1]
        push!(result, byte)
    end
    return join(result)
end

# function Base.readavailable(sp::SerialPort)

#     while Base.eof(sp) == false
#         # Block if no data is available
#     end

#     return Base.readall(sp)
# end
