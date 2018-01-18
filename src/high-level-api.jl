
mutable struct SerialPort <: IO
    ref::Port
    eof::Bool
    open::Bool
    function SerialPort(ref, eof, open)
        sp = new(ref, eof, open)
        finalizer(sp, destroy!)
        return sp
    end
end

"""
`sp = SerialPort(portname::AbstractString)`

Constructor for the `SerialPort` object.
"""
SerialPort(portname::AbstractString) = SerialPort(sp_get_port_by_name(portname), false, false)

"""
`destroy!(sp::SerialPort)`

Destructor for the `SerialPort` object.
"""
function destroy!(sp::SerialPort)
    close(sp)
    sp_free_port(sp.ref)
end

"""
`set_speed(sp::SerialPort,bps::Integer)`

Set connection speed of `sp` in bits per second. The library will return an
error if bps is not a valid/supported value.
"""
function set_speed(sp::SerialPort, bps::Integer)
     sp_set_baudrate(sp.ref, bps)
     return nothing
end

"""
`set_frame(sp::SerialPort [, ndatabits::Integer, pariry::SPParity, nstopbits::Integer])`

Configure packet framing. Defaults to the most common "8N1" scheme. See
https://en.wikipedia.org/wiki/Universal_asynchronous_receiver/transmitter#Data_framing
for more details.

`ndatabits` is the number of data bits which is `8` in the common "8N1" sceme.

The `parity` is set to none in the "8N1" sceme and can take the values:
`SP_PARITY_NONE`, `SP_PARITY_ODD`, `SP_PARITY_EVEN`, `SP_PARITY_MARK` and
`SP_PARITY_SPACE`.

`nstopbits` is the number of stop bits, which is `1` by default.
"""
function set_frame(sp::SerialPort;
    ndatabits::Integer=8,
    parity::SPParity=SP_PARITY_NONE,
    nstopbits::Integer=1)

    sp_set_bits(sp.ref, ndatabits)
    sp_set_parity(sp.ref, parity)
    sp_set_stopbits(sp.ref, nstopbits)
    return nothing
end

"""
`set_flow_control(sp::SerialPort [,rts::SPrts, cts::SPcts, dtr::SPdtr, dst::SPdsr, xonxoff::SPXonXoff])`

Configure flow control settings. Many systems don't support all options.
If an unsupported option is requested, the library will return SP_ERR_SUPP.

`rts` can take the values: `SP_RTS_OFF`, `SP_RTS_ON` and `SP_RTS_FLOW_CONTROL`
and defaults to `SP_RTS_OFF`.

`cts` can take the values: `SP_CTS_IGNORE` and `SP_CTS_FLOW_CONTROL`. Its
default is `SP_CTS_IGNORE`.

`dtr` can take the values: `SP_DTR_OFF`, `SP_DTR_ON`, and `SP_DTR_FLOW_CONTROL`
and defaults to `SP_DTR_OFF`.

`dsr` can take the values: `SP_DSR_IGNORE` and `SP_DSR_FLOW_CONTROL`. Its
default is SP_DSR_IGNORE`.

`xonxoff` can take values: `SP_XONXOFF_DISABLED`, `SP_XONXOFF_IN`,
`SP_XONXOFF_OUT`, and `SP_XONXOFF_INOUT` and defaults to `SP_XONXOFF_DISABLED`.
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
    return nothing
end

"""
`listports([nports_guess::Integer])`

Print a list of currently visible ports, along with some basic info.

`nports_guess` provides the number of ports guessed. Its default is `64`.
"""
function list_ports(;nports_guess::Integer=64)
    ports = sp_list_ports()

    for port in unsafe_wrap(Array, ports, nports_guess, false)
        port == C_NULL && return

        println(sp_get_port_name(port))
        println("\tDescription:\t",    sp_get_port_description(port))
        println("\tTransport type:\t", sp_get_port_transport(port))
    end

    sp_free_port_list(ports)
    return nothing
end


"""
`print_port_metadata(sp::SerialPort [,show_config::Bool])

Print info found for this port.
Note: port should be open to obtain a valid FD/handle before accessing fields.

`show_config` is `true` by default and prints out the current port settings.
"""
function print_port_metadata(sp::SerialPort; show_config::Bool=true)
    print_port_metadata(sp.ref, show_config=show_config)
    return nothing
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
    return nothing
end

"""
`print_port_settings(sp::SerialPort)`

Print port settings for `sp`.
"""
print_port_settings(sp::SerialPort) = print_port_settings(sp.ref)

function print_port_settings(port::LibSerialPort.Port)
    println("Configuration for ", sp_get_port_name(port), ":")
    config = sp_get_config(port)
    print_port_settings(config)
    sp_free_config(config)
end

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

"""
`open(sp::SerialPort [, mode::SPMode])`

Open the serial port `sp`.

`mode` can take the values: `SP_MODE_READ`, `SP_MODE_WRITE`, and
`SP_MODE_READ_WRITE`
"""
function Base.open(sp::SerialPort; mode::SPMode=SP_MODE_READ_WRITE)
    sp_open(sp.ref, mode)
    sp.open = true
    return sp
end

"""
`open(portname::AbstractString,baudrate::Integer [,mode::SPMode,
    ndatabits::Integer,parity::SPParity,nstopbits::Integer])`

construct, configure and open a `SerialPort` object.

For details on posssible settings see `?set_flow_control` and `?set_frame`.
"""
function Base.open(portname::AbstractString,
                   bps::Integer;
                   mode::SPMode=SP_MODE_READ_WRITE,
                   ndatabits::Integer=8,
                   parity::SPParity=SP_PARITY_NONE,
                   nstopbits::Integer=1)
    sp = SerialPort(sp_get_port_by_name(portname), false, true)
    sp_open(sp.ref, mode)
    set_speed(sp, bps)
    set_frame(sp, ndatabits=ndatabits, parity=parity, nstopbits=nstopbits)
    return sp
end

"""
`open_serial_port(port_address::AbstractString, baudrate::Integer)`

Create and configure a SerialPort object with the standard 8N1 settings
and specified `baudrate`. Example: `open_serial_port("/dev/ttyACM0", 115200)`
"""
function open_serial_port(port_address::AbstractString, baudrate::Integer)
    sp = SerialPort(port_address)
    open(sp)
    set_speed(sp, baudrate)
    set_frame(sp, ndatabits=8, parity=SP_PARITY_NONE, nstopbits=1)
    return sp
end

"""
close(sp::SerialPort)

Close the serial port `sp`.
"""
function Base.close(sp::SerialPort)
    if sp.open
        # Flush first, as is done in other close() methods in Base
        sp_flush(sp.ref, SP_BUF_BOTH)

        sp_close(sp.ref)
    end
    return sp
end

"""
`flush(sp::SerialPort [, buffer::SPBuffer])`

Flush `buffer` of serial port `sp`.

`buffer` can take the values: `SP_BUF_INPUT`, `SP_BUF_OUTPUT`, and
`SP_BUF_BOTH`.
"""
function Base.flush(sp::SerialPort; buffer::SPBuffer=SP_BUF_BOTH)
    sp_flush(sp.ref, buffer)
end

"""
`write(sp::SerialPort, data::String)`
`write(sp::SerialPort, data::Array{Char})`
`write(sp::SerialPort, data::Array{UInt8})`

Write sequence of Bytes to `sp`.
"""
function Base.write(sp::SerialPort, data::Array{UInt8})
    sp_nonblocking_write(sp.ref, data)
    return sp_drain(sp.ref)
end

function Base.write(sp::SerialPort, data::Array{Char})
    sp_nonblocking_write(sp.ref, data)
    return sp_drain(sp.ref)
end

function Base.write(sp::SerialPort, data::String)
    sp_nonblocking_write(sp.ref, convert(Array{UInt8},data))
    return sp_drain(sp.ref)
end

"""
`write(sp::SerialPort, data::UInt8)`
`write(sp::SerialPort, data::Char)`

Write single Byte to `sp`
"""
Base.write(sp::SerialPort, data::Char) = write(sp, [data])
Base.write(sp::SerialPort, data::UInt8) = write(sp, [data])

"""
`write(sp::SerialPort, i::Integer)`

Write string representation of `i` to `sp`.
"""
Base.write(sp::SerialPort, i::Integer) = Base.write(sp, "$i")

"""
`write(sp::SerialPort, f::AbstractFloat)`
`write(sp::SerialPort, f::AbstractFloat, format::AbstractString)`

Write formatted string representation of `f` to `sp`. By default the string is
formated using `format="%.3f"`. For details on the format consult the
documentation of the C library function `sprintf`.
"""
Base.write(sp::SerialPort, f::AbstractFloat, format::AbstractString="%.3f") = Base.write(sp, eval(:@sprintf($format, $f)))

"""
`eof(sp::SerialPort)`

Return EOF state (`true` or `false`) of `sp`.
"""
Base.eof(sp::SerialPort) = sp.eof

"""
`seteof(sp::SerialPort, state::Bool)`

Set EOF of `sp` to `state`
"""
function seteof(sp::SerialPort, state::Bool)
    sp.eof = state
    return nothing
end

"""
`seteof(sp::SerialPort, state::Bool)`

Reset EOF of `sp` to `false`
"""
reseteof(sp::SerialPort) = seteof(sp, false)

"""
`read(sp::SerialPort, T::Type{UInt8})`
`read(sp::SerialPort, T::Type{Char})`

Read a single Byte from the specified port and return it represented as `T`.
`T` might be either `Char or `UInt8`. if no Byte is availible in the port
buffer return zero.
"""
function Base.read(sp::SerialPort, readType::Type{Char})
    nbytes_read, bytes = sp_nonblocking_read(sp.ref, 1)
    return (nbytes_read == 1) ? convert(readType,bytes[1]) : readType(0)
end

function Base.read(sp::SerialPort, readType::Type{UInt8})
    nbytes_read, bytes = sp_nonblocking_read(sp.ref, 1)
    return (nbytes_read == 1) ? convert(readType,bytes[1]) : readType(0)
end

"""
`readuntil(sp::SerialPort,delim::Union{Char,AbstractString,Vector{Char}},timeout_ms::Integer)`

Read until the specified delimiting byte (e.g. `'\\n'`) is encountered, or until
timeout_ms has elapsed, whichever comes first.
"""
function Base.readuntil(sp::SerialPort, delim::Char, timeout_ms::Real)
    return readuntil(sp,[delim],timeout_ms)
end


function Base.readuntil(sp::SerialPort, delim::AbstractString, timeout_ms::Real)
    return readuntil(sp,convert(Vector{Char},delim),timeout_ms)
end

function Base.readuntil(sp::SerialPort, delim::Vector{Char}, timeout_ms::Real)
    start_time = time_ns()
    out = IOBuffer()
    lastchars = Char[0 for i=1:length(delim)]
    while !eof(sp)
        if (time_ns() - start_time)/1e6 > timeout_ms
            break
        end
        if nb_available(sp) > 0
            c = read(sp, Char)
            write(out, c)
            lastchars = circshift(lastchars,-1)
            lastchars[end] = c
            if lastchars == delim
                break
            end
        end
    end
    return String(take!(out))
end

"""
`Base.nb_available(sp::SerialPort)`

Gets the number of bytes waiting in the input buffer.
"""
Base.nb_available(sp::SerialPort) = Int(sp_input_waiting(sp.ref))

"""
`readbytes!(sp::SerialPort,nbytes::Integer)`

Read `nbytes` from the specified serial port `sp`, without blocking. Returns
a `UInt8` `Array`.
"""
function Base.readbytes!(sp::SerialPort, nbytes::Integer)
    nbytes_read, bytes = sp_nonblocking_read(sp.ref, nbytes)
    return bytes
end

"""
`readstring(sp::SerialPort)`

Read everything from the specified serial ports `sp` input buffer, one byte at
a time, until it is empty. Returns a `Char` array.
"""
function Base.readstring(sp::SerialPort)
    result = Char[]
    while Int(nb_available(sp)) > 0
        byte = readbytes!(sp, 1)[1]
        push!(result, byte)
    end
    return join(result)
end
