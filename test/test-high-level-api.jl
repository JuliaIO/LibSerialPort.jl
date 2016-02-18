
using LibSerialPort

function test_nonblocking_serial_loopback(sp::SerialPort)

    println("\n[TEST] Read any incoming data for ~1 second...")
    for i = 1:1000
        print(readall(sp))
        sleep(0.001)
    end
    println()

    flush(sp, buffer=SP_BUF_BOTH)

    print("\n\n[TEST] Serial loopback - ")
    println("Send 100 short messages and read whatever comes back...")

    tic()
    for i = 1:100
        write(sp, "Test message $i\n")
        print(readall(sp))
    end

    sleep(0.1)

    # Read and print any remaining data
    while nb_available(sp) > 0
        print(readall(sp))
    end
    println()
    toc()

    flush(sp, buffer=SP_BUF_BOTH)
end

function main()
    sp = SerialPort("/dev/cu.wchusbserial1410")
    open(sp)
    set_speed(sp, 19200)
    set_frame(sp, ndatabits=8, parity=SP_PARITY_NONE, nstopbits=1)

    # print_port_info(sp)

    test_nonblocking_serial_loopback(sp)

    close(sp, delete=true)
end

main()