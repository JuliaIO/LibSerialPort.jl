"""Minimal working example.
"""

using LibSerialPort

function main(args)
    if length(args) != 2
        println("Usage: $(basename(@__FILE__)) port baudrate")
        println("Available ports:")
        list_ports()
        return
    end
	portname, baudrate = args

	baudrate = parse(Int, baudrate)

	LibSerialPort.open(portname, baudrate) do sp
		sleep(2)

		if bytesavailable(sp) > 0
        	println(String(read(sp)))
    	end

	    write(sp, "hello\n")
	    sleep(0.1)
	    println(readline(sp))
	end

end

main(ARGS)
