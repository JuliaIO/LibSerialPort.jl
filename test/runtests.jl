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

if haskey(ENV, "CI")
    @testset "LibSerialPort CI port listing" begin
        list_ports()
    end
else
    if length(ARGS) == 0
        port = "/dev/ttyS0"  # /dev/ttyS4 /dev/ttyUSB0
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

        @testset "Reading with timeouts" begin
            LibSerialPort.open(port, 115200) do s
                #Tests assume serial port being tested won't output anything after being flushed
                #TODO: Find a better way to test this
                flush(s)
                @test readline(s, 1.0) == "" #readline with a 1 second timeout
                @test readuntil(s, 'a', 1.0) == "" #readuntil 'a' with a 1 second timeout
            end
        end

        # console.jl runs forever, thus isn't amenable to unit testing
        # @testset "Examples" begin
        #     include("../examples/console.jl")
        #     @test console() == nothing
        #     @test console(port, baudrate) == nothing
        # end
    end
end
