[![](https://img.shields.io/badge/docs-stable-blue.svg)](https://JuliaIO.github.io/LibSerialPort.jl/stable) [![](https://img.shields.io/badge/docs-dev-blue.svg)](https://JuliaIO.github.io/LibSerialPort.jl/dev) [![Build Status](https://travis-ci.com/JuliaIO/LibSerialPort.jl.svg?branch=master)](https://travis-ci.com/JuliaIO/LibSerialPort.jl)

# LibSerialPort.jl

[libserialport](http://sigrok.org/wiki/Libserialport) is a small, well-documented C library for general-purpose serial port communication. This is a julia wrapper for the library.

Apart from a very few non-essential functions, the entire library API (about 75 functions) is wrapped using `ccall`. In addition, a higher-level interface is also provided that follows Julia's IO stream interface.

## Installation

    pkg> add LibSerialPort


## Usage

Try

    julia> using LibSerialPort
    julia> list_ports()  # or get_port_list() for an array of names

The examples/ directory contains a simple serial console for the command line. This may serve as a useful starting point for your application. The serial_example.ino sketch can be flashed to a microcontroller supported by the Arduino environment.

```julia
using LibSerialPort

# Modify these as needed
portname = "/dev/ttyS0"
baudrate = 115200

# Snippet from examples/mwe.jl
LibSerialPort.open(portname, baudrate) do sp
	sleep(2)

	if bytesavailable(sp) > 0
    	println(String(read(sp)))
	end

    write(sp, "hello\n")
    sleep(0.1)
    println(readline(sp))
end
```

The tests are also worth looking at for demonstration of i/o and configuration. They can be run via `julia test/runtests.jl <address> <baudrate>`. Unless the address of your device matches that in runtests.jl, doing `pkg> test LibSerialPort` will fail. This problem would be addressed by [support for args](https://github.com/JuliaLang/Pkg.jl/issues/518) in the Pkg REPL.

Note that on Windows, returning an OS-level port handle is not yet supported.
