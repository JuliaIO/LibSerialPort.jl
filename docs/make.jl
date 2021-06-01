using Documenter, LibSerialPort, LibSerialPort.Lib

makedocs(
    sitename="LibSerialPort.jl",
    format = Documenter.HTML(
        prettyurls = get(ENV, "CI", nothing) == "true"
    )
)

deploydocs(
    repo = "github.com/JuliaIO/LibSerialPort.jl.git",
)
