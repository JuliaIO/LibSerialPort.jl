# TODO: Windows.

using BinDeps

const version = "0.1.1"
const src_uri = "http://sigrok.org/download/source/libserialport/libserialport-" * version * ".tar.gz"

@BinDeps.setup

deps = [
    libserialport = library_dependency("libserialport", aliases = ["serialport", "libserialport.0"])
]

provides(Sources, Dict(URI(src_uri) => libserialport))

prefix = joinpath(BinDeps.depsdir(libserialport),"usr")
srcdir = joinpath(BinDeps.depsdir(libserialport),"src", "libserialport-" * version)

provides(SimpleBuild,
    (@build_steps begin
        GetSources(libserialport)
        @build_steps begin
            ChangeDirectory(srcdir)
            `./configure --prefix=$prefix`
            `make install`
        end
    end), libserialport, installed_libpath=joinpath(prefix, "lib"))


@BinDeps.install Dict(:libserialport => :libserialport)
