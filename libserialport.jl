
@enum(SPReturn,
    SP_OK = 0,
    SP_ERR_ARG = -1,
    SP_ERR_FAIL = -2,
    SP_ERR_MEM = -3,
    SP_ERR_SUPP = -4)

@enum(SPMode,
    SP_MODE_READ = 1,
    SP_MODE_WRITE = 2,
    SP_MODE_READ_WRITE = 3)

@enum(SPEvent,
    SP_EVENT_RX_READY = 1,
    SP_EVENT_TX_READY = 2,
    SP_EVENT_ERROR = 4)

@enum(SPBuffer,
    SP_BUF_INPUT = 1,
    SP_BUF_OUTPUT = 2,
    SP_BUF_BOTH = 3)

@enum(SPParity,
    SP_PARITY_INVALID = -1,
    SP_PARITY_NONE = 0,
    SP_PARITY_ODD = 1,
    SP_PARITY_EVEN = 2,
    SP_PARITY_MARK = 3,
    SP_PARITY_SPACE = 4)

@enum(SPrts,
    SP_RTS_INVALID = -1,
    SP_RTS_OFF = 0,
    SP_RTS_ON = 1,
    SP_RTS_FLOW_CONTROL = 2)

@enum(SPcts,
    SP_CTS_INVALID = -1,
    SP_CTS_IGNORE = 0,
    SP_CTS_FLOW_CONTROL = 1)

@enum(SPdtr,
    SP_DTR_INVALID = -1,
    SP_DTR_OFF = 0,
    SP_DTR_ON = 1,
    SP_DTR_FLOW_CONTROL = 2)

@enum(SPdsr,
    SP_DSR_INVALID = -1,
    SP_DSR_IGNORE = 0,
    SP_DSR_FLOW_CONTROL = 1)

@enum(SPXonXoff,
    SP_XONXOFF_INVALID = -1,
    SP_XONXOFF_DISABLED = 0,
    SP_XONXOFF_IN = 1,
    SP_XONXOFF_OUT = 2,
    SP_XONXOFF_INOUT = 3)

@enum(SPFlowControl,
    SP_FLOWCONTROL_NONE = 0,
    SP_FLOWCONTROL_XONXOFF = 1,
    SP_FLOWCONTROL_RTSCTS = 2,
    SP_FLOWCONTROL_DTRDSR = 3)

@enum(SPSignal,
    SP_SIG_CTS = 1,
    SP_SIG_DSR = 2,
    SP_SIG_DCD = 4,
    SP_SIG_RI = 8)

@enum(SPTransport,
    SP_TRANSPORT_NATIVE,
    SP_TRANSPORT_USB,
    SP_TRANSPORT_BLUETOOTH)

type SPEventSet
    handles::Ptr{Void}
    masks::Ptr{SPEvent}
    count::Cuint
end

function notify_on_error(ret::SPReturn)
    ret >= SP_OK && return

    msg = "libserialport returned $ret - "

    if ret == SP_ERR_ARG
        msg *= "Function was called with invalid arguments."
    elseif ret == SP_ERR_FAIL
        msg *= "Host OS reported a failure. Error code/message provided by the OS "
        msg *= "can be obtained by calling sp_last_error_code() or sp_last_error_message()."
    elseif ret == SP_ERR_MEM
        msg *= "Memory allocation failed."
    elseif ret == SP_ERR_SUPP
        msg *= "No support for the requested operation in the current OS, driver or device."
    else
        error("Unknown SPReturn value")
    end

    error(msg)
end

# enum sp_return sp_get_port_by_name(const char *portname, struct sp_port **port_ptr);
function sp_get_port_by_name(portname::AbstractString)
    portref = Ref{Ptr{Void}}()
    ret = ccall((:sp_get_port_by_name, "libserialport"), SPReturn,
                (Ptr{UInt8}, Ref{Ptr{Void}}), portname, portref)
    notify_on_error(ret)
    portref[]
end

# void sp_free_port(struct sp_port *port);
function sp_free_port(port::Ref{Void})
    ccall((:sp_free_port, "libserialport"), Void, (Ref{Void},), port)
end

# enum sp_return sp_list_ports(struct sp_port ***list_ptr);
function sp_list_ports()
    ports = Ref{Ptr{Ptr{Void}}}()
    ret = ccall((:sp_list_ports, "libserialport"),
                SPReturn, (Ref{Ptr{Ptr{Void}}},), ports)
    notify_on_error(ret)
    return ports[]
end

# enum sp_return sp_copy_port(const struct sp_port *port, struct sp_port **copy_ptr);
function sp_copy_port(port::Ref{Void})
    port_copy = Ref{Ptr{Void}}()
    ret = ccall((:sp_copy_port, "libserialport"), SPReturn,
                (Ref{Void}, Ref{Ptr{Void}}), port, port_copy)
    notify_on_error(ret)
    return port_copy[]
end

# void sp_free_port_list(struct sp_port **ports);
function sp_free_port_list(ports::Ref{Ptr{Void}})
    ccall((:sp_free_port_list, "libserialport"), Void, (Ref{Ptr{Void}},), ports)
end

# enum sp_return sp_open(struct sp_port *port, enum sp_mode flags);
function sp_open(port::Ref{Void}, mode::SPMode)
    ret = ccall((:sp_open, "libserialport"), SPReturn, (Ref{Void}, SPMode), port, mode)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_close(struct sp_port *port);
function sp_close(port::Ref{Void})
    ret = ccall((:sp_close, "libserialport"), SPReturn, (Ref{Void},), port)
    notify_on_error(ret)
    ret
end

# char *sp_get_port_name(const struct sp_port *port);
function sp_get_port_name(port::Ref{Void})
    cname = ccall((:sp_get_port_name, "libserialport"), Ptr{UInt8}, (Ref{Void},), port)
    name = cname != C_NULL ? bytestring(cname) : ""
end

# char *sp_get_port_description(const struct sp_port *port);
function sp_get_port_description(port::Ref{Void})
    d = ccall((:sp_get_port_description, "libserialport"), Ptr{UInt8}, (Ref{Void},), port)
    desc = d != C_NULL ? bytestring(d) : ""
end

# enum sp_transport sp_get_port_transport(const struct sp_port *port);
function sp_get_port_transport(port::Ref{Void})
    ccall((:sp_get_port_transport, "libserialport"), SPTransport, (Ref{Void},), port)
end

# enum sp_return sp_get_port_usb_bus_address(const struct sp_port *port, int *usb_bus, int *usb_address);
function sp_get_port_usb_bus_address(port::Ref{Void})

    if sp_get_port_transport(port) != SP_TRANSPORT_USB
        warn("Port does not use USB transport")
        return
    end

    usb_bus = Ref{Cint}()
    usb_address = Ref{Cint}()
    ret = ccall((:sp_get_port_usb_bus_address, "libserialport"), SPReturn,
                (Ref{Void}, Ref{Cint}, Ref{Cint}), port, usb_bus, usb_address)

    if ret == SP_ERR_SUPP
        return -1, -1
    end

    notify_on_error(ret)
    return usb_bus[], usb_address[]
end

# enum sp_return sp_get_port_usb_vid_pid(const struct sp_port *port, int *usb_vid, int *usb_pid);
function sp_get_port_usb_vid_pid(port::Ref{Void})

    if sp_get_port_transport(port) != SP_TRANSPORT_USB
        warn("Port does not use USB transport")
        return
    end

    vid = Ref{Cint}()
    pid = Ref{Cint}()
    ret = ccall((:sp_get_port_usb_vid_pid, "libserialport"), SPReturn,
                (Ref{Void}, Ref{Cint}, Ref{Cint}), port, vid, pid)

    if ret == SP_ERR_SUPP
        return -1, -1
    end

    notify_on_error(ret)
    return vid[], pid[]
end

# char *sp_get_port_usb_manufacturer(const struct sp_port *port);
function sp_get_port_usb_manufacturer(port::Ref{Void})
    m = ccall((:sp_get_port_usb_manufacturer, "libserialport"),
              Ptr{UInt8}, (Ref{Void},), port)
    manufacturer = (m != C_NULL) ? bytestring(m) : ""
end

# char *sp_get_port_usb_product(const struct sp_port *port);
function sp_get_port_usb_product(port::Ref{Void})
    p = ccall((:sp_get_port_usb_product, "libserialport"),
              Ptr{UInt8}, (Ref{Void},), port)
    product = (p != C_NULL) ? bytestring(p) : ""
end

# char *sp_get_port_usb_serial(const struct sp_port *port);
function sp_get_port_usb_serial(port::Ref{Void})
    s = ccall((:sp_get_port_usb_serial, "libserialport"),
              Ptr{UInt8}, (Ref{Void},), port)
    serial = (s != C_NULL) ? bytestring(s) : ""
end

# char *sp_get_port_bluetooth_address(const struct sp_port *port);
function sp_get_port_bluetooth_address(port::Ref{Void})
    a = ccall((:sp_get_port_bluetooth_address, "libserialport"),
              Ptr{UInt8}, (Ref{Void},), port)
    address = (a != C_NULL) ? bytestring(a) : ""
end

# enum sp_return sp_get_port_handle(const struct sp_port *port, void *result_ptr);
function sp_get_port_handle(port::Ref{Void})
    # For Linux and OS X
    result = Ref{Cint}(0)

    # TODO: on Windows, result should be Ref{HANDLE}

    ret = ccall((:sp_get_port_handle, "libserialport"), SPReturn,
                (Ref{Void}, Ref{Cint}), port, result)
    notify_on_error(ret)
    result[]
end

# enum sp_return sp_new_config(struct sp_port_config **config_ptr);
function sp_new_config()
    pc = Ref{Ptr{Void}}()
    ret = ccall((:sp_new_config, "libserialport"), SPReturn,
                (Ref{Ptr{Void}},), pc)
    notify_on_error(ret)
    pc[]
end

# void sp_free_config(struct sp_port_config *config);
function sp_free_config(config::Ref{Void})
    ccall((:sp_free_config, "libserialport"), Void, (Ref{Void},), config)
end

function sp_get_config(port::Ref{Void})
    config = sp_new_config()
    ret = ccall((:sp_get_config, "libserialport"), SPReturn,
                (Ref{Void}, Ref{Void}), port, config)
    notify_on_error(ret)
    config
end

# enum sp_return sp_set_config(struct sp_port *port, const struct sp_port_config *config);
function sp_set_config(port::Ref{Void}, config::Ref{Void})
    ret = ccall((:sp_set_config, "libserialport"), SPReturn,
                (Ref{Void}, Ref{Void}), port, config)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_set_baudrate(struct sp_port *port, int baudrate);
function sp_set_baudrate(port::Ref{Void}, baudrate::Integer)
    ret = ccall((:sp_set_baudrate, "libserialport"), SPReturn,
                (Ref{Void}, Cint), port, Cint(baudrate))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_get_config_baudrate(const struct sp_port_config *config, int *baudrate_ptr);
function sp_get_config_baudrate(config::Ref{Void})
    baudrate = Ref{Cint}()
    ret = ccall((:sp_get_config_baudrate, "libserialport"), SPReturn,
                (Ref{Void}, Ref{Cint}), config, baudrate)
    notify_on_error(ret)
    baudrate[]
end

# enum sp_return sp_set_config_baudrate(struct sp_port_config *config, int baudrate);
function sp_set_config_baudrate(config::Ref{Void}, baudrate::Integer)
    ret = ccall((:sp_set_config_baudrate, "libserialport"), SPReturn,
                (Ref{Void}, Cint), config, Cint(baudrate))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_set_bits(struct sp_port *port, int bits);
function sp_set_bits(port::Ref{Void}, bits::Integer)
    @assert 5 <= bits <= 8
    ret = ccall((:sp_set_bits, "libserialport"), SPReturn,
                (Ref{Void}, Cint), port, Cint(bits))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_get_config_bits(const struct sp_port_config *config, int *bits_ptr);
function sp_get_config_bits(config::Ref{Void})
    bits = Ref{Cint}()
    ret = ccall((:sp_get_config_bits, "libserialport"), SPReturn,
                (Ref{Void}, Ref{Cint}), config, bits)
    notify_on_error(ret)
    bits[]
end

# enum sp_return sp_set_config_bits(struct sp_port_config *config, int bits);
function sp_set_config_bits(config::Ref{Void}, bits::Integer)
    ret = ccall((:sp_set_config_bits, "libserialport"), SPReturn,
                (Ref{Void}, Cint), config, Cint(bits))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_set_parity(struct sp_port *port, enum sp_parity parity);
function sp_set_parity(port::Ref{Void}, parity::SPParity)
    ret = ccall((:sp_set_parity, "libserialport"), SPReturn,
                (Ref{Void}, SPParity), port, parity)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_get_config_parity(const struct sp_port_config *config, enum sp_parity *parity_ptr);
function sp_get_config_parity(config::Ref{Void})
    parity = Ref{SPParity}()
    ret = ccall((:sp_get_config_parity, "libserialport"), SPReturn,
                (Ref{Void}, Ref{SPParity}), config, parity)
    notify_on_error(ret)
    parity[]
end

# enum sp_return sp_set_config_parity(struct sp_port_config *config, enum sp_parity parity);
function sp_set_config_parity(config::Ref{Void}, parity::SPParity)
    ret = ccall((:sp_set_config_parity, "libserialport"), SPReturn,
                (Ref{Void}, SPParity), config, parity)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_set_stopbits(struct sp_port *port, int stopbits);
function sp_set_stopbits(port::Ref{Void}, stopbits::Integer)
    ret = ccall((:sp_set_stopbits, "libserialport"), SPReturn,
                (Ref{Void}, Cint), port, Cint(stopbits))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_get_config_stopbits(const struct sp_port_config *config, int *stopbits_ptr);
function sp_get_config_stopbits(config::Ref{Void})
    bits = Ref{Cint}()
    ret = ccall((:sp_get_config_stopbits, "libserialport"), SPReturn,
                (Ref{Void}, Ref{Cint}), config, bits)
    notify_on_error(ret)
    bits[]
end

# enum sp_return sp_set_config_stopbits(struct sp_port_config *config, int stopbits);
function sp_set_config_stopbits(config::Ref{Void}, stopbits::Integer)
    ret = ccall((:sp_set_config_stopbits, "libserialport"), SPReturn,
                (Ref{Void}, Cint), config, Cint(stopbits))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_set_rts(struct sp_port *port, enum sp_rts rts);
function sp_set_rts(port::Ref{Void}, rts::SPrts)
    ret = ccall((:sp_set_rts, "libserialport"), SPReturn,
                (Ref{Void}, SPrts), port, rts)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_get_config_rts(const struct sp_port_config *config, enum sp_rts *rts_ptr);
function sp_get_config_rts(config::Ref{Void})
    rts = Ref{SPrts}()
    ret = ccall((:sp_get_config_rts, "libserialport"), SPReturn,
                (Ref{Void}, Ref{SPrts}), config, rts)
    notify_on_error(ret)
    rts[]
end

# enum sp_return sp_set_config_rts(struct sp_port_config *config, enum sp_rts rts);
function sp_set_config_rts(config::Ref{Void}, rts::SPrts)
    ret = ccall((:sp_set_config_rts, "libserialport"), SPReturn,
                (Ref{Void}, SPrts), config, SPrts(rts))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_set_cts(struct sp_port *port, enum sp_cts cts);
function sp_set_cts(port::Ref{Void}, cts::SPcts)
    ret = ccall((:sp_set_cts, "libserialport"), SPReturn,
                (Ref{Void}, SPcts), port, cts)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_get_config_cts(const struct sp_port_config *config, enum sp_cts *cts_ptr);
function sp_get_config_cts(config::Ref{Void})
    cts = Ref{SPcts}()
    ret = ccall((:sp_get_config_cts, "libserialport"), SPReturn,
                (Ref{Void}, Ref{SPcts}), config, cts)
    notify_on_error(ret)
    cts[]
end

# enum sp_return sp_set_config_cts(struct sp_port_config *config, enum sp_cts cts);
function sp_set_config_cts(config::Ref{Void}, cts::SPcts)
    ret = ccall((:sp_set_config_cts, "libserialport"), SPReturn,
                (Ref{Void}, SPcts), config, SPcts(cts))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_set_dtr(struct sp_port *port, enum sp_dtr dtr);
function sp_set_dtr(port::Ref{Void}, dtr::SPdtr)
    ret = ccall((:sp_set_dtr, "libserialport"), SPReturn,
                (Ref{Void}, SPdtr), port, dtr)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_get_config_dtr(const struct sp_port_config *config, enum sp_dtr *dtr_ptr);
function sp_get_config_dtr(config::Ref{Void})
    dtr = Ref{SPdtr}()
    ret = ccall((:sp_get_config_dtr, "libserialport"), SPReturn,
                (Ref{Void}, Ref{SPdtr}), config, dtr)
    notify_on_error(ret)
    dtr[]
end

# enum sp_return sp_set_config_dtr(struct sp_port_config *config, enum sp_dtr dtr);
function sp_set_config_dtr(config::Ref{Void}, dtr::SPdtr)
    ret = ccall((:sp_set_config_dtr, "libserialport"), SPReturn,
                (Ref{Void}, SPdtr), config, SPdtr(dtr))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_set_dsr(struct sp_port *port, enum sp_dsr dsr);
function sp_set_dsr(port::Ref{Void}, dsr::SPdsr)
    ret = ccall((:sp_set_dsr, "libserialport"), SPReturn,
                (Ref{Void}, SPdsr), port, dsr)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_get_config_dsr(const struct sp_port_config *config, enum sp_dsr *dsr_ptr);
function sp_get_config_dsr(config::Ref{Void})
    dsr = Ref{SPdsr}()
    ret = ccall((:sp_get_config_dsr, "libserialport"), SPReturn,
                (Ref{Void}, Ref{SPdsr}), config, dsr)
    notify_on_error(ret)
    dsr[]
end

# enum sp_return sp_set_config_dsr(struct sp_port_config *config, enum sp_dsr dsr);
function sp_set_config_dsr(config::Ref{Void}, dsr::SPdsr)
    ret = ccall((:sp_set_config_dsr, "libserialport"), SPReturn,
                (Ref{Void}, SPdsr), config, SPdsr(dsr))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_set_xon_xoff(struct sp_port *port, enum sp_xonxoff xon_xoff);
function sp_set_xon_xoff(port::Ref{Void}, xon_xoff::SPXonXoff)
    ret = ccall((:sp_set_xon_xoff, "libserialport"), SPReturn,
                (Ref{Void}, SPXonXoff), port, xon_xoff)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_get_config_xon_xoff(const struct sp_port_config *config, enum sp_xonxoff *xon_xoff_ptr);
function sp_get_config_xon_xoff(config::Ref{Void})
    xon_xoff = Ref{SPXonXoff}()
    ret = ccall((:sp_get_config_xon_xoff, "libserialport"), SPReturn,
                (Ref{Void}, Ref{SPXonXoff}), config, xon_xoff)
    notify_on_error(ret)
    xon_xoff[]
end

# enum sp_return sp_set_config_xon_xoff(struct sp_port_config *config, enum sp_xonxoff xon_xoff);
function sp_set_config_xon_xoff(config::Ref{Void}, xon_xoff::SPXonXoff)
    ret = ccall((:sp_set_config_xon_xoff, "libserialport"), SPReturn,
                (Ref{Void}, SPXonXoff), config, SPXonXoff(xon_xoff))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_set_config_flowcontrol(struct sp_port_config *config, enum sp_flowcontrol flowcontrol);
function sp_set_config_flowcontrol(config::Ref{Void}, flowcontrol::SPFlowControl)
    ret = ccall((:sp_set_config_flowcontrol, "libserialport"), SPReturn,
                (Ref{Void}, SPFlowControl), config, flowcontrol)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_set_flowcontrol(struct sp_port *port, enum sp_flowcontrol flowcontrol);
function sp_set_flowcontrol(port::Ref{Void}, flowcontrol::SPFlowControl)
    ret = ccall((:sp_set_flowcontrol, "libserialport"), SPReturn,
                (Ref{Void}, SPFlowControl), port, flowcontrol)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_blocking_read(struct sp_port *port, void *buf, size_t count, unsigned int timeout_ms);
function sp_blocking_read(port::Ref{Void}, nbytes::Integer)
    buffer = Array(UInt8, nbytes)

    ret = ccall((:sp_blocking_read, "libserialport"), SPReturn,
                (Ref{Void}, Ptr{UInt8}, Csize_t, Cuint),
                port, buffer, sizeof(buffer), Cuint(1000))
    notify_on_error(ret)

    return bytestring(pointer(buffer))
end

# enum sp_return sp_blocking_read_next(struct sp_port *port, void *buf, size_t count, unsigned int timeout_ms);

# enum sp_return sp_nonblocking_read(struct sp_port *port, void *buf, size_t count);
function sp_nonblocking_read(port::Ref{Void}, nbytes::Integer)
    buffer = Array(UInt8, nbytes)
    ret = ccall((:sp_nonblocking_read, "libserialport"), SPReturn,
                (Ref{Void}, Ptr{UInt8}, Csize_t), port, buffer, sizeof(buffer))
    notify_on_error(ret)

    buffer[end] = 0
    return bytestring(pointer(buffer))
end

# function sp_nonblocking_read(port::Ref{Void})
#     # buffer = Ref{Void}()

#     nbytes = Csize_t(1024)
#     # buffer = Array(UInt8, 1024)
#     buffer = Ref{Ptr{UInt8}}()

#     ret = ccall((:sp_nonblocking_read, "libserialport"), SPReturn,
#                 (Ref{Void}, Ref{Ptr{UInt8}}, Csize_t), port, buffer, nbytes)
#     # ret = ccall((:sp_nonblocking_read, "libserialport"), SPReturn,
#     #             (Ref{Void}, Array{UInt8}, Csize_t), port, buffer, nbytes)
#     notify_on_error(ret)

#     buffer[]
#     # bytestring(buffer[])
# end

# enum sp_return sp_blocking_write(struct sp_port *port, const void *buf, size_t count, unsigned int timeout_ms);
function sp_blocking_write(port::Ref{Void}, buffer::Array{UInt8}, timeout_ms::Integer)
    ret = ccall((:sp_blocking_write, "libserialport"), SPReturn,
                (Ref{Void}, Ptr{UInt8}, Csize_t, Cuint),
                port, pointer(buffer), sizeof(buffer), Cuint(timeout_ms))
    notify_on_error(ret)
    ret
end


# function sp_nonblocking_write(port::Ref{Void}, msg::Array{UInt8}, nbytes::Integer)
#     buffer = pointer(msg)
#     ret = ccall((:sp_nonblocking_write, "libserialport"), SPReturn,
#                 (Ref{Void}, Ptr{UInt8}, Csize_t), port, buffer, Csize_t(nbytes))
#     notify_on_error(ret)
#     ret
# end

# enum sp_return sp_nonblocking_write(struct sp_port *port, const void *buf, size_t count);
function sp_nonblocking_write(port::Ref{Void}, buffer::Array{UInt8})
    ret = ccall((:sp_nonblocking_write, "libserialport"), SPReturn,
                (Ref{Void}, Ptr{UInt8}, Csize_t), port, pointer(buffer), sizeof(buffer))
    notify_on_error(ret)
    ret
end

# enum sp_return sp_input_waiting(struct sp_port *port);
"""
Returns the number of bytes in the input buffer or an error code.
"""
function sp_input_waiting(port::Ref{Void})
    ret = ccall((:sp_input_waiting, "libserialport"), SPReturn, (Ref{Void},), port)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_output_waiting(struct sp_port *port);
"""
Returns the number of bytes in the output buffer or an error code.
"""
function sp_output_waiting(port::Ref{Void})
    ret = ccall((:sp_output_waiting, "libserialport"), SPReturn, (Ref{Void},), port)
    notify_on_error(ret)
    ret
end

# enum sp_return sp_flush(struct sp_port *port, enum sp_buffer buffers);
# enum sp_return sp_drain(struct sp_port *port);
# enum sp_return sp_new_event_set(struct sp_event_set **result_ptr);
# enum sp_return sp_add_port_events(struct sp_event_set *event_set, const struct sp_port *port, enum sp_event mask);
# enum sp_return sp_wait(struct sp_event_set *event_set, unsigned int timeout_ms);
# void sp_free_event_set(struct sp_event_set *event_set);
# enum sp_return sp_get_signals(struct sp_port *port, enum sp_signal *signal_mask);
# enum sp_return sp_start_break(struct sp_port *port);
# enum sp_return sp_end_break(struct sp_port *port);
# int sp_last_error_code(void);
# char *sp_last_error_message(void);
# void sp_free_error_message(char *message);
# void sp_set_debug_handler(void (*handler)(const char *format, ...));
# void sp_default_debug_handler(const char *format, ...);
# int sp_get_major_package_version(void);
# int sp_get_minor_package_version(void);
# int sp_get_micro_package_version(void);
# const char *sp_get_package_version_string(void);
# int sp_get_current_lib_version(void);
# int sp_get_revision_lib_version(void);
# int sp_get_age_lib_version(void);
# const char *sp_get_lib_version_string(void);
