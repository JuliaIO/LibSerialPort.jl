# LibSerialPort.jl – access serial ports

```@docs
LibSerialPort
```

# Enumerating serial ports

```@docs
list_ports
get_port_list
print_port_metadata
```

## Opening and configuring ports

```@docs
LibSerialPort.open(::AbstractString, ::Integer)
SerialPort(::AbstractString)
open(::SerialPort; ::SPMode)
close(sp::SerialPort)
set_speed
set_frame
set_flow_control
isopen(sp::SerialPort)
eof(sp::SerialPort)
seteof
get_port_settings
print_port_settings
set_read_timeout
set_write_timeout
clear_read_timeout
clear_write_timeout
sp_flush
sp_drain
sp_output_waiting
```

# Read and write methods from Base

Many of the read/write methods defined in `Base` that operate on an
object of type `IO` can also be used with objects of type
`SerialPort`. Therefore, also consult the documentation for

```
Base.read(::IO, ::Any)
Base.read!
Base.readbytes!
Base.readchomp
Base.readavailable
Base.readline
Base.readlines
Base.eachline
Base.write
Base.print(::IO, ::Any)
```

# Additional read methods

```@docs
read(::SerialPort)
nonblocking_read(::SerialPort)
bytesavailable(::SerialPort)
```
