# LibSerialPort.jl

[libserialport](http://sigrok.org/wiki/Libserialport) is a small, well-documented C library for general-purpose serial port communication. This is a julia wrapper for the library.

Apart from a very few non-essential functions, the entire library API (about 75 functions) is wrapped using `ccall`. In addition, a higher-level interface is also provided that follows Julia's IO stream interface (note: subject to change soon).


# Dependencies

[BinDeps.jl](https://github.com/JuliaLang/BinDeps.jl) is required, as well as standard tools for building the C library (make, gcc/clang, etc).


# Installation

This package is not in the Julia package registry. Our plan is to merge it into [SerialPorts.jl](https://github.com/JuliaIO/SerialPorts.jl). For now, it can be cloned via

    julia> Pkg.clone("https://github.com/andrewadare/LibSerialPort.jl.git")

To install the C library, do `Pkg.build`:

    julia> Pkg.build("LibSerialPort")

(This is only set up fo Unix-like systems. PRs welcome for Windows). Alternatively, follow the [build instructions](http://sigrok.org/wiki/Libserialport) for a system-wide build. If installation through the package system succeeded, then

    julia> readdir(joinpath(Pkg.dir("LibSerialPort"), "deps/usr/lib"))

should list your new library. Type

    julia> using LibSerialPort
    julia> list_ports()

to get a list of ports detected on your system.

There is currently no documentation, but the examples/ and tests/ directories contain some examples.
