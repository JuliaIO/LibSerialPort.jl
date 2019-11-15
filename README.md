[![Build Status](https://travis-ci.com/JuliaIO/LibSerialPort.jl.svg?branch=master)](https://travis-ci.com/JuliaIO/LibSerialPort.jl)

# LibSerialPort.jl

[libserialport](http://sigrok.org/wiki/Libserialport) is a small, well-documented C library for general-purpose serial port communication. This is a julia wrapper for the library.

Apart from a very few non-essential functions, the entire library API (about 75 functions) is wrapped using `ccall`. In addition, a higher-level interface is also provided that follows Julia's IO stream interface.

## Installation

    pkg> add LibSerialPort


## Usage

Try

    julia> using LibSerialPort
    julia> list_ports()

to get a list of ports detected on your system.

The examples/ directory contains a simple serial console for the command line. This may serve as a useful starting point for your application. The serial_example.ino sketch can be flashed to a microcontroller supported by the Arduino environment.


The tests are also worth looking at for demonstration of i/o and configuration. They can be run via `julia test/runtests.jl <address> <baudrate>`. Unless the address of your device matches that in runtests.jl, doing `pkg> test LibSerialPort` will fail. This problem would be addressed by [support for args](https://github.com/JuliaLang/Pkg.jl/issues/518) in the Pkg REPL.

Note that on Windows, returning an OS-level port handle is not yet supported.

### Reading with timeouts

Methods for `readline` and `readuntil` are exported that allow for a timeout in seconds.

i.e.
```julia
LibSerialPort.open(port, 115200) do s
    write(s, "input")
    ret = readline(s, 1.0) #try to readline with a 1 second timeout
    @show ret
end
```

```julia
LibSerialPort.open(port, 115200) do s
    write(s, "input")
    ret = readuntil(s, 'x', 1.0) #try to readuntil char `x` with a 1 second timeout
    @show ret
end
```
