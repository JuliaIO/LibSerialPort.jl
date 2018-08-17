#=
Run test locally using
$ julia test/runtest.jl /dev/ttyXYZ

/dev/ttyXYZ can be:
/dev/ttyS0  (for Travis)
/dev/ttyS4
/dev/ttyUSB0
=#

using LibSerialPort
using Test


if length(ARGS) == 0
    port = "/dev/ttyS0"  # /dev/ttyS4 /dev/ttyUSB0
else
    port = ARGS[1]
end

@testset "LibSerialPort" begin
    @testset "Low level API" begin
        include("test-low-level-api.jl")
        @test test_low_level_api() == nothing
        @test test_low_level_api(port) == nothing
    end


    @testset "High level API" begin
        include("test-high-level-api.jl")
        @test test_high_level_api() == nothing
        @test test_high_level_api(port) == nothing
    end

    @testset "Examples" begin
        include("../examples/console.jl")
        @test console() == nothing
        @test console(port) == nothing
    end
end
