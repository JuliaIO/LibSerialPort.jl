[![Build Status](https://travis-ci.com/andrewadare/LibSerialPort.jl.svg?branch=master)](https://travis-ci.com/andrewadare/LibSerialPort.jl)

# LibSerialPort.jl

[libserialport](http://sigrok.org/wiki/Libserialport) is a small, well-documented C library for general-purpose serial port communication. This is a julia wrapper for the library.

Apart from a very few non-essential functions, the entire library API (about 75 functions) is wrapped using `ccall`. In addition, a higher-level interface is also provided that follows Julia's IO stream interface.


# Dependencies

[BinDeps.jl](https://github.com/JuliaLang/BinDeps.jl) is required, as well as standard tools for building the C library (make, gcc/clang, etc).


# Installation

    pkg> add LibSerialPort

To install the C library, use `Pkg.build`:

    pkg> build LibSerialPort

On Unix-like systems, `libserialport` will be built from source. On Windows a pre-built shared library is downloaded and installed into the package directory. Alternatively, follow the [build instructions](http://sigrok.org/wiki/Libserialport) for a system-wide build. If installation through the package system succeeded, then

    julia> readdir(joinpath(Pkg.dir("LibSerialPort"), "deps/usr/lib"))

should list your new library.

# Usage

Try

    julia> using LibSerialPort
    julia> list_ports()

to get a list of ports detected on your system.

The examples/ directory contains a simple serial console for the command line. This may serve as a useful starting point for your application. The serial_example.ino sketch can be flashed to a microcontroller supported by the Arduino environment. The tests are also worth looking at for demonstration of i/o and configuration. They can be run via `julia test/runtests.jl <address> <baudrate>`. Unless the address of your device matches that in runtests.jl, doing `pkg> test LibSerialPort` will fail. This problem would be addressed by [support for args](https://github.com/JuliaLang/Pkg.jl/issues/518) in the Pkg REPL.

Note that on Windows, returning an OS-level port handle is not yet supported.
