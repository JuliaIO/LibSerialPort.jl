import Base: isopen, open, close, write, unsafe_write, flush,
    read, unsafe_read, bytesavailable, eof


mutable struct SerialPort <: IO
    ref::Port
    is_eof::Bool
    is_open::Bool

    # cumulative timeout limits, passed on to blocking libserialport functions
    # 0 means wait indefinitely, as per sigrok libserialport interface
    read_timeout_ms::Cuint
    write_timeout_ms::Cuint

    function SerialPort(ref, is_eof, is_open)
        sp = new(ref, is_eof, is_open, 0, 0)
        finalizer(destroy!, sp)
        return sp
    end
end


"""
A `Timeout` exception is thrown by blocking read or write functions
when the cumulative blocking time since that last call to
`set_read_timeout(t)` or `set_write_timeout(t)` has exceeded `t`
seconds.
"""
struct Timeout <: Exception
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


function get_port_settings(config::LibSerialPort.Config)
    return Dict(
        "baudrate" => sp_get_config_baudrate(config),
        "bits"     => sp_get_config_bits(config),
        "parity"   => sp_get_config_parity(config),
        "stopbits" => sp_get_config_stopbits(config),
        "RTS"      => sp_get_config_rts(config),
        "CTS"      => sp_get_config_cts(config),
        "DTR"      => sp_get_config_dtr(config),
        "DSR"      => sp_get_config_dsr(config),
        "XonXoff"  => sp_get_config_xon_xoff(config),
        )
end

function get_port_settings(port::LibSerialPort.Port)
    config = sp_get_config(port)
    settings = get_port_settings(config)
    sp_free_config(config)
    return settings
end

"""
`get_port_settings(sp::SerialPort)`

Return port settings for `sp` as a dictionary.
"""
get_port_settings(sp::SerialPort) = get_port_settings(sp.ref)


"""
`print_port_settings(sp::SerialPort)`

Print port settings for `sp`.
"""
function print_port_settings(sp::SerialPort)
    print_port_settings(sp.ref)
    println("Read timeout (ms, forever if 0): ", sp.read_timeout_ms)
    return
end

function print_port_settings(port::LibSerialPort.Port)
    println("Configuration for ", sp_get_port_name(port), ":")
    config = sp_get_config(port)
    print_port_settings(config)
    sp_free_config(config)
    return
end

function print_port_settings(config::LibSerialPort.Config)
    s = get_port_settings(config)
    for (k, v) in s
        println("\t$k\t", v)
    end
    return
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
    return
end

# fixes https://github.com/JuliaIO/LibSerialPort.jl/issues/53
@deprecate flush(sp::SerialPort, buffer::SPBuffer=SP_BUF_BOTH) sp_flush(sp, buffer)

# pass through some methods from the low-level interface
sp_output_waiting(sp::SerialPort) = sp_output_waiting(sp.ref)
sp_flush(sp::SerialPort, args...) = sp_flush(sp.ref, args...)
sp_drain(sp::SerialPort)          = sp_drain(sp.ref)

"""
    set_read_timeout(sp::SerialPort, seconds::Real)

Set a read timeout limit of `t` > 0 seconds for the total (cumulative)
time that subsequently called blocking read functions can wait before
a `Timeout` exception is thrown.

# Example

```
sp=LibSerialPort.open("/dev/ttyUSB0", 115200)
# wait until either two lines have been received
# or 10 seconds have elapsed
set_read_timeout(sp, 10)
try
    line1 = readuntil(sp, '\n')
    line2 = readuntil(sp, '\n')
catch e
    if isa(e, LibSerialPort.Timeout)
        println("Too late!")
    else
        rethrow()
    end
end
clear_read_timeout(sp)
```

See also: [`clear_read_timeout`](@ref), [`set_write_timeout`](@ref)
"""
function set_read_timeout(sp::SerialPort, seconds::Real)
    @assert seconds > 0
    sp.read_timeout_ms =  seconds * 1000;
    return
end

"""
    set_write_timeout(sp::SerialPort, seconds::Real)

Set a write timeout limit of `t` > 0 seconds for the total (cumulative)
time that subsequently called blocking read functions can wait before
a `Timeout` exception is thrown.

# Example

```
sp=LibSerialPort.open("/dev/ttyUSB0", 300)
# wait until either 4000 periods have been
# passed on to the serial-port driver or
# 10 seconds have elapsed
set_write_timeout(sp, 10)
try
    for i=1:50 ; write(sp, '.' ^ 80); end
catch e
    if isa(e, LibSerialPort.Timeout)
        println("This took too long!")
    else
        rethrow()
    end
end
clear_write_timeout(sp)
```

See also: [`clear_write_timeout`](@ref), [`set_read_timeout`](@ref)
"""
function set_write_timeout(sp::SerialPort, seconds::Real)
    @assert seconds > 0
    sp.write_timeout_ms =  seconds * 1000;
    return
end

"""
    clear_read_timeout(sp::SerialPort)

Cancel any previous read timeout, such that blocking read operations
will now wait without any time limit.
"""
function clear_read_timeout(sp::SerialPort)
    sp.read_timeout_ms = 0;
    return
end

"""
    clear_write_timeout(sp::SerialPort)

Cancel any previous write timeout, such that blocking write
operations will block without any time limit.
"""
function clear_write_timeout(sp::SerialPort)
    sp.write_timeout_ms = 0;
    return
end

# We define here only the basic methods for reading and writing
# bytes. All other methods for reading/writing the canonical binary
# representation of any type, and print() methods for writing
# its text representation, are inherited from the IO supertype
# (see julia/base/io.jl), i.e. work just like for files.

function read(sp::SerialPort, ::Type{UInt8})
    byte_ref = Ref{UInt8}(0)
    unsafe_read(sp, byte_ref, 1)
    return byte_ref[]
end


function unsafe_read(sp::SerialPort, p::Ptr{UInt8}, nb::UInt)
    if sp.read_timeout_ms > 0
        starttime = time_ns()
    end
    nbytes = sp_blocking_read(sp.ref, p, nb, sp.read_timeout_ms)
    if nbytes < nb
        @assert sp.read_timeout_ms > 0
        sp.read_timeout_ms = 0
        throw(Timeout())
    elseif sp.read_timeout_ms > 0
        # how much of the timeout have we used up already?
        elapsed_ms = (time_ns() - starttime + 999_999) ÷ 1_000_000
        if sp.read_timeout_ms > elapsed_ms
            sp.read_timeout_ms -= elapsed_ms
        else
            sp.read_timeout_ms = 0
            throw(Timeout())
        end
    end
    nothing
end


function write(sp::SerialPort, b::UInt8)
    unsafe_write(sp, Ref(b), 1)
end


function unsafe_write(sp::SerialPort, p::Ptr{UInt8}, nb::UInt)
    if sp.write_timeout_ms > 0
        starttime = time_ns()
    end
    nbytes = sp_blocking_write(sp.ref, p, nb, sp.write_timeout_ms)
    if nbytes < nb
        @assert sp.write_timeout_ms > 0
        sp.write_timeout_ms = 0
        throw(Timeout())
    elseif sp.write_timeout_ms > 0
        # how much of the timeout have we used up already?
        elapsed_ms = (time_ns() - starttime + 999_999) ÷ 1_000_000
        if sp.write_timeout_ms > elapsed_ms
            sp.write_timeout_ms -= elapsed_ms
        else
            sp.write_timeout_ms = 0
            throw(Timeout())
        end
    end
    nbytes
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
bytesavailable(sp::SerialPort) = sp_input_waiting(sp.ref)

