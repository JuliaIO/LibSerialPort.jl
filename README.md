# LibSerialPort.jl

[libserialport](http://sigrok.org/wiki/Libserialport) is a small, well-documented c library for general-purpose serial port communication. This is a julia wrapper for the library. You can use it for things like microcontroller communication, [war games](http://www.imdb.com/media/rm542161664/tt0086567?ref_=ttmi_mi_all_sf_2), and conversing with pretty much anything sporting a UART peripheral. This package provides an alternative to [SerialPorts.jl](https://github.com/sjkelly/SerialPorts.jl) that allows fine-grained configuration, doesn't depend on Python, and supports both blocking and non-blocking i/o.

Apart from a very few non-essential functions, the entire library API (about 75 functions) is wrapped using `ccall`. In addition, a higher-level interface is also provided that follows Julia's IO stream interface (note: this is not yet complete).

# Installation

This package is not yet registered. Get it via

    julia> Pkg.clone("https://github.com/andrewadare/LibSerialPort.jl.git")

For a system-wide installation of the library, follow the simple [build instructions](http://sigrok.org/wiki/Libserialport), or download and build using `Pkg.build`:

    julia> Pkg.build("LibSerialPort")

(This only works on Unix-like systems, for now). If installation through the package system succeeded, then

    julia> readdir(joinpath(Pkg.dir("LibSerialPort"), "deps/usr/lib"))

should list your new library. Type

    julia> using LibSerialPort
    julia> list_ports()

to get a list of ports detected on your system.

# Using the package

Further documentation is forthcoming, but for now, the tests directory contains some useful examples.
