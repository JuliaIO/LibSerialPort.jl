# LibSerialPort.jl

[libserialport](http://sigrok.org/wiki/Libserialport) is a small, well-written, and well-documented c library for general-purpose serial port communication. This is a julia wrapper for the libserialport c library. You can use it for things like microcontroller communication, [war games](http://www.imdb.com/media/rm542161664/tt0086567?ref_=ttmi_mi_all_sf_2), and talking to vintage hardware from Julia.

# Installation

This package is not yet registered. Get it via

    Pkg.clone("https://github.com/andrewadare/LibSerialPort.jl.git")

and then do

    Pkg.build("LibSerialPort")

which should download and build the c library (at least on Unix-like systems, for now). If installation was successful, try doing

    sp_list_ports()

# Using the package
Further documentation is forthcoming, but for now, the tests directory contains some useful examples.
