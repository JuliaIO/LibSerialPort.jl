# TODO: Windows.

using BinDeps

const version = "0.1.1"
const src_uri = "http://sigrok.org/download/source/libserialport/libserialport-" * version * ".tar.gz"

@BinDeps.setup

libserialport = library_dependency("libserialport", aliases = ["serialport", "libserialport.0", "libserialport-0"])

if is_windows()
  version == "0.1.1" || error("Only version 0.1.1 available for Windows")
  win32_bin_uri = "https://ci.appveyor.com/api/buildjobs/1t0y955tdpvlnl27/artifacts/libserialport-0.1.1.zip"
  win64_bin_uri = "https://ci.appveyor.com/api/buildjobs/1t0y955tdpvlnl27/artifacts/libserialport-0.1.1.zip"
  provides(Binaries, URI(Sys.ARCH == :x86_64 ? win64_bin_uri : win32_bin_uri),libserialport, os = :Windows)
else
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
      end), libserialport)
end

@BinDeps.install Dict(:libserialport => :libserialport)
