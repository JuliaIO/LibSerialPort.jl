#=
Run test locally using
$ julia test/runtests.jl /dev/ttyXYZ

/dev/ttyXYZ can be:
/dev/ttyS0  (for Travis)
/dev/ttyS4
/dev/ttyUSB0
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
