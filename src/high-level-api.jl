import Base: isopen, open, close, write, unsafe_write, flush,
    read, unsafe_read, bytesavailable, eof


mutable struct SerialPort <: IO
    ref::Port
    is_eof::Bool
    is_open::Bool
    read_timeout_ms::Int  # 0 to wait indefinitely, per sigrok libserialport interface
    function SerialPort(ref, is_eof, is_open, read_timeout_ms)
        sp = new(ref, is_eof, is_open, read_timeout_ms)
        finalizer(destroy!, sp)
        return sp
    end
end


"""
`sp = SerialPort(portname::AbstractString)`

Constructor for the `SerialPort` object.
"""
SerialPort(portname::AbstractString) = SerialPort(sp_get_port_by_name(portname), false, false, 0)


"""
`destroy!(sp::SerialPort)`

Destructor for the `SerialPort` object.
"""
function destroy!(sp::SerialPort)
    close(sp)
    sp_free_port(sp.ref)
end


"""
`isopen(sp::SerialPort) -> Bool`

Determine whether a SerialPort object is open.
"""
isopen(sp::SerialPort) = sp.is_open


"""
`eof(sp::SerialPort) -> Bool`

Return EOF state (`true` or `false`)`.
"""
eof(sp::SerialPort) = sp.is_eof


"""
`set_speed(sp::SerialPort,bps::Integer)`

Set connection speed of `sp` in bits per second. Raise an
`ErrorException` if `bps` is not a valid/supported value.
"""
function set_speed(sp::SerialPort, bps::Integer)
     sp_set_baudrate(sp.ref, bps)
     return nothing
end


"""
`set_frame(sp::SerialPort [, ndatabits::Integer, parity::SPParity, nstopbits::Integer])`

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

    for port in unsafe_wrap(Array, ports, nports_guess, own=false)
        port == C_NULL && return

        println(sp_get_port_name(port))
        println("\tDescription:\t",    sp_get_port_description(port))
        println("\tTransport type:\t", sp_get_port_transport(port))
    end

    sp_free_port_list(ports)
    return nothing
end


"""
`get_port_list([nports_guess::Integer])`

Return a vector of currently visible ports.

`nports_guess` provides the number of ports guessed. Its default is `64`.
"""
function get_port_list(;nports_guess::Integer=64)
    ports = sp_list_ports()
    port_list = String[]
    for port in unsafe_wrap(Array, ports, nports_guess, own=false)
        port == C_NULL && return port_list
        push!(port_list, sp_get_port_name(port))
    end
    sp_free_port_list(ports)
    return port_list
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
    transport = sp_get_port_transport(port)
    print("\nPort transport:\t");
    if transport == SP_TRANSPORT_NATIVE
        println("native serial port")
    elseif transport == SP_TRANSPORT_USB
        println("USB")
        println("Manufacturer:\t",      sp_get_port_usb_manufacturer(port))
        println("Product:\t",           sp_get_port_usb_product(port))
        println("USB serial number:\t", sp_get_port_usb_serial(port))
        bus, addr = sp_get_port_usb_bus_address(port)
        println("USB bus #:\t", bus)
        println("Address on bus:\t", addr)
        vid, pid = sp_get_port_usb_vid_pid(port)
        println("Vendor ID:\t", vid)
        println("Product ID:\t", pid)
    elseif transport == SP_TRANSPORT_BLUETOOTH
        println("Bluetooth")
        println("Bluetooth address:\t", sp_get_port_bluetooth_address(port))
    end
    println("File descriptor:\t",   sp_get_port_handle(port))

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
function open(sp::SerialPort; mode::SPMode=SP_MODE_READ_WRITE)
    sp_open(sp.ref, mode)
    sp.is_open = true
    return sp
end


"""
`open(portname::AbstractString,baudrate::Integer [,mode::SPMode,
    ndatabits::Integer,parity::SPParity,nstopbits::Integer])`

construct, configure and open a `SerialPort` object.

For details on posssible settings see `?set_flow_control` and `?set_frame`.
"""
function open(portname::AbstractString,
              bps::Integer;
              mode::SPMode=SP_MODE_READ_WRITE,
              ndatabits::Integer=8,
              parity::SPParity=SP_PARITY_NONE,
              nstopbits::Integer=1)
    sp = SerialPort(portname)
    sp_open(sp.ref, mode)
    sp.is_open = true
    set_speed(sp, bps)
    set_frame(sp, ndatabits=ndatabits, parity=parity, nstopbits=nstopbits)
    return sp
end


"""
close(sp::SerialPort)

Close the serial port `sp`.
"""
function close(sp::SerialPort)
    if isopen(sp)
        sp_close(sp.ref)
        sp.is_open = false
    end
    return sp
end

# fixes https://github.com/JuliaIO/LibSerialPort.jl/issues/53
@deprecate flush(sp::SerialPort, buffer::SPBuffer=SP_BUF_BOTH) sp_flush(sp, buffer)

# pass through some methods from the low-level interface
sp_output_waiting(sp::SerialPort) = sp_output_waiting(sp.ref)
sp_flush(sp::SerialPort, args...) = sp_flush(sp.ref, args...)
sp_drain(sp::SerialPort)          = sp_drain(sp.ref)

# We define here only the basic methods for writing bytes.
# All other write() methods for writing the canonical binary
# representation of any type, and print() methods for writing
# its text representation, are inherited from the IO supertype
# (see julia/base/io.jl), i.e. work just like for files.

function write(sp::SerialPort, b::UInt8)
    Int(sp_blocking_write(sp.ref, Ref(b)))
end

function unsafe_write(sp::SerialPort, p::Ptr{UInt8}, nb::UInt)
    Int(sp_blocking_write(sp.ref, p, nb))
end


"""
`seteof(sp::SerialPort, state::Bool)`

Set EOF of `sp` to `state`
"""
function seteof(sp::SerialPort, state::Bool)
    sp.is_eof = state
    return nothing
end


"""
`reseteof(sp::SerialPort, state::Bool)`

Reset EOF of `sp` to `false`
"""
reseteof(sp::SerialPort) = seteof(sp, false)


# Julia's Base module defines `read(s::IO, nb::Integer = typemax(Int))`.
# Override the default `nb` to a more useful value for this context.
read(sp::SerialPort) = read(sp, bytesavailable(sp))


function read(sp::SerialPort, ::Type{UInt8})
    byte_ref = Ref{UInt8}(0)
    sp_blocking_read(sp.ref, byte_ref, 1, sp.read_timeout_ms)
    return byte_ref.x
end


function unsafe_read(sp::SerialPort, p::Ptr{UInt8}, nb::UInt)
    sp_blocking_read(sp.ref, p, nb, sp.read_timeout_ms)
end


"""
`nonblocking_read(sp::SerialPort)`

Read everything from the specified serial ports `sp` input buffer, one byte at
a time, until it is empty. Returns a `String`.
"""
function nonblocking_read(sp::SerialPort)
    result = UInt8[]
    byte_ref = Ref{UInt8}(0)
    while bytesavailable(sp) > 0
        sp_nonblocking_read(sp.ref, byte_ref, 1)
        push!(result, byte_ref.x)
    end
    return result
end


"""
`bytesavailable(sp::SerialPort)`

Gets the number of bytes waiting in the input buffer.
"""
bytesavailable(sp::SerialPort) = Int(sp_input_waiting(sp.ref))

