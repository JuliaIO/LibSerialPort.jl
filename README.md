# LibSerialPort.jl

[libserialport](http://sigrok.org/wiki/Libserialport) is a small, well-documented c library for general-purpose serial port communication. This is a julia wrapper for the library. You can use it for things like microcontroller communication, [war games](http://www.imdb.com/media/rm542161664/tt0086567?ref_=ttmi_mi_all_sf_2), and talking to vintage hardware from Julia.

# Installation

This package is not yet registered. Get it via

    Pkg.clone("https://github.com/andrewadare/LibSerialPort.jl.git")

For a system-wide installation of the library, follow the simple [build instructions](http://sigrok.org/wiki/Libserialport), or download and build using BinDeps as follows:

    Pkg.build("LibSerialPort")

(This only works on Unix-like systems, for now). If installation was successful, try doing

    using LibSerialPort
    list_ports()

to get a list of ports detected on your system.

# Using the package

Further documentation is forthcoming, but for now, the tests directory contains some useful examples.

# Troubleshooting

If the libserialport shared library was built correctly with `Pkg.build`, but you receive an error that the library could not be loaded, one option is to add the `lib` directory to your `(DY)LD_LIBRARY_PATH`.
