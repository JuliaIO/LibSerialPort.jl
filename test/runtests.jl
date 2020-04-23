#=
These tests require a serial device to echo bytes written from the host computer.

For example, these hardware configurations should work:
 - A standalone USB-to-UART adapter (e.g. FTDI FT232) in loopback mode (TX and RX pins jumped).
 - A USB-to-serial equipped, arduino-compatible microcontroller running examples/serial_example.ino.
 - The USB-to-serial chip on some microcontroller boards can be used directly, bypassing the micro.
   For example, an Arduino UNO R3 can be connected via USB with RESET grounded and TX wired to RX.

Run the tests locally using
$ julia test/runtests.jl <port address> <baudrate>

LibSerialPort.list_ports() may help with finding the correct port address.
=#

using LibSerialPort
using Test

if haskey(ENV, "CI")
    @testset "LibSerialPort CI port listing" begin
        list_ports()
    end
else
    if length(ARGS) == 0
        port = "/dev/ttyS0"
    else
        port = ARGS[1]
    end

    baudrate = length(ARGS) >= 2 ? ARGS[2] : "115200"

    @testset "LibSerialPort" begin
        @testset "Low level API" begin
            include("test-low-level-api.jl")
            @test test_low_level_api() == nothing
            @test test_low_level_api(port, baudrate) == nothing
        end

        @testset "High level API" begin
            include("test-high-level-api.jl")
            @test test_high_level_api() == nothing
            @test test_high_level_api(port, baudrate) == nothing
        end

    end
end
