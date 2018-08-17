using LibSerialPort
using Test

@testset "LibSerialPort" begin
    port = "/dev/ttyS4"

    @testset "Low level API" begin
        include("test-low-level-api.jl")
        test_low_level_api()
        test_low_level_api(port)
    end


    @testset "High level API" begin
        include("test-high-level-api.jl")
        test_high_level_api()
        test_high_level_api(port)
    end

    @testset "Examples" begin
        include("../examples/console.jl")
        console()
        console(port)
    end
end