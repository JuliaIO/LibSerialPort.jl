"""
The `LibSerialPort.Lib` module provides a low-level Julia wrapper
around each of the C function, types and constants exported by the C
library [libserialport](http://sigrok.org/wiki/Libserialport). The
wrapper functions very closely follow the C API, but they check for
error codes returned and raise these as a Julia `ErrorException`.
"""
module Lib

using Libdl
using libserialport_jll

export
    # Types
    Port,
    Config,

    # Enum types
    SPReturn,
    SPMode,
    SPEvent,
    SPBuffer,
    SPParity,
    SPrts,
    SPcts,
    SPdtr,
    SPdsr,
    SPXonXoff,
    SPFlowControl,
    SPSignal,
    SPTransport,

    # Enum values
    SP_OK,
    SP_ERR_ARG,
    SP_ERR_FAIL,
    SP_ERR_MEM,
    SP_ERR_SUPP,
    SP_MODE_READ,
    SP_MODE_WRITE,
    SP_MODE_READ_WRITE,
    SP_EVENT_RX_READY,
    SP_EVENT_TX_READY,
    SP_EVENT_ERROR,
    SP_BUF_INPUT,
    SP_BUF_OUTPUT,
    SP_BUF_BOTH,
    SP_PARITY_INVALID,
    SP_PARITY_NONE,
    SP_PARITY_ODD,
    SP_PARITY_EVEN,
    SP_PARITY_MARK,
    SP_PARITY_SPACE,
    SP_RTS_INVALID,
    SP_RTS_OFF,
    SP_RTS_ON,
    SP_RTS_FLOW_CONTROL,
    SP_CTS_INVALID,
    SP_CTS_IGNORE,
    SP_CTS_FLOW_CONTROL,
    SP_DTR_INVALID,
    SP_DTR_OFF,
    SP_DTR_ON,
    SP_DTR_FLOW_CONTROL,
    SP_DSR_INVALID,
    SP_DSR_IGNORE,
    SP_DSR_FLOW_CONTROL,
    SP_XONXOFF_INVALID,
    SP_XONXOFF_DISABLED,
    SP_XONXOFF_IN,
    SP_XONXOFF_OUT,
    SP_XONXOFF_INOUT,
    SP_FLOWCONTROL_NONE,
    SP_FLOWCONTROL_XONXOFF,
    SP_FLOWCONTROL_RTSCTS,
    SP_FLOWCONTROL_DTRDSR,
    SP_SIG_CTS,
    SP_SIG_DSR,
    SP_SIG_DCD,
    SP_SIG_RI,
    SP_TRANSPORT_NATIVE,
    SP_TRANSPORT_USB,
    SP_TRANSPORT_BLUETOOTH,

    # Functions from libserialport API
    sp_get_port_by_name,
    sp_free_port,
    sp_list_ports,
    sp_copy_port,
    sp_free_port_list,
    sp_open,
    sp_close,
    sp_get_port_name,
    sp_get_port_description,
    sp_get_port_transport,
    sp_get_port_usb_bus_address,
    sp_get_port_usb_vid_pid,
    sp_get_port_usb_manufacturer,
    sp_get_port_usb_product,
    sp_get_port_usb_serial,
    sp_get_port_bluetooth_address,
    sp_get_port_handle,
    sp_new_config,
    sp_free_config,
    sp_get_config,
    sp_set_config,
    sp_set_baudrate,
    sp_get_config_baudrate,
    sp_set_config_baudrate,
    sp_set_bits,
    sp_get_config_bits,
    sp_set_config_bits,
    sp_set_parity,
    sp_get_config_parity,
    sp_set_config_parity,
    sp_set_stopbits,
    sp_get_config_stopbits,
    sp_set_config_stopbits,
    sp_set_rts,
    sp_get_config_rts,
    sp_set_config_rts,
    sp_set_cts,
    sp_get_config_cts,
    sp_set_config_cts,
    sp_set_dtr,
    sp_get_config_dtr,
    sp_set_config_dtr,
    sp_set_dsr,
    sp_get_config_dsr,
    sp_set_config_dsr,
    sp_set_xon_xoff,
    sp_get_config_xon_xoff,
    sp_set_config_xon_xoff,
    sp_set_config_flowcontrol,
    sp_set_flowcontrol,
    sp_blocking_read,
    sp_blocking_read_next,
    sp_nonblocking_read,
    sp_blocking_write,
    sp_nonblocking_write,
    sp_input_waiting,
    sp_output_waiting,
    sp_flush,
    sp_drain,
    sp_new_event_set,
    sp_add_port_events,
    sp_wait,
    sp_free_event_set,
    sp_get_signals,
    sp_start_break,
    sp_end_break,
    sp_last_error_code,
    sp_last_error_message,
    sp_get_major_package_version,
    sp_get_minor_package_version,
    sp_get_micro_package_version,
    sp_get_package_version_string,
    sp_get_current_lib_version,
    sp_get_revision_lib_version,
    sp_get_age_lib_version,
    sp_get_lib_version_string

# Julia types for the sp_port, sp_port_config, and sp_event_set structs
struct SPPort end
struct SPConfig end
struct SPEventSet end

const Port = Ref{SPPort}
const PortP = Ref{Ptr{SPPort}}
const Config = Ref{SPConfig}
const ConfigP = Ref{Ptr{SPConfig}}

@enum SPReturn begin
    SP_OK = 0
    SP_ERR_ARG = -1
    SP_ERR_FAIL = -2
    SP_ERR_MEM = -3
    SP_ERR_SUPP = -4
end

@enum SPMode begin
    SP_MODE_READ = 1
    SP_MODE_WRITE = 2
    SP_MODE_READ_WRITE = 3
end

@enum SPEvent begin
    SP_EVENT_RX_READY = 1
    SP_EVENT_TX_READY = 2
    SP_EVENT_ERROR = 4
end

@enum SPBuffer begin
    SP_BUF_INPUT = 1
    SP_BUF_OUTPUT = 2
    SP_BUF_BOTH = 3
end

@enum SPParity begin
    SP_PARITY_INVALID = -1
    SP_PARITY_NONE = 0
    SP_PARITY_ODD = 1
    SP_PARITY_EVEN = 2
    SP_PARITY_MARK = 3
    SP_PARITY_SPACE = 4
end

@enum SPrts begin
    SP_RTS_INVALID = -1
    SP_RTS_OFF = 0
    SP_RTS_ON = 1
    SP_RTS_FLOW_CONTROL = 2
end

@enum SPcts begin
    SP_CTS_INVALID = -1
    SP_CTS_IGNORE = 0
    SP_CTS_FLOW_CONTROL = 1
end

@enum SPdtr begin
    SP_DTR_INVALID = -1
    SP_DTR_OFF = 0
    SP_DTR_ON = 1
    SP_DTR_FLOW_CONTROL = 2
end

@enum SPdsr begin
    SP_DSR_INVALID = -1
    SP_DSR_IGNORE = 0
    SP_DSR_FLOW_CONTROL = 1
end

@enum SPXonXoff begin
    SP_XONXOFF_INVALID = -1
    SP_XONXOFF_DISABLED = 0
    SP_XONXOFF_IN = 1
    SP_XONXOFF_OUT = 2
    SP_XONXOFF_INOUT = 3
end

@enum SPFlowControl begin
    SP_FLOWCONTROL_NONE = 0
    SP_FLOWCONTROL_XONXOFF = 1
    SP_FLOWCONTROL_RTSCTS = 2
    SP_FLOWCONTROL_DTRDSR = 3
end

@enum SPSignal begin
    SP_SIG_CTS = 1
    SP_SIG_DSR = 2
    SP_SIG_DCD = 4
    SP_SIG_RI = 8
end

@enum SPTransport begin
    SP_TRANSPORT_NATIVE
    SP_TRANSPORT_USB
    SP_TRANSPORT_BLUETOOTH
end

# Define constructors to make conversion to Int explicit, since falling back to
# `convert` from a missing constructor is now deprecated behavior.
SPReturn(x::SPReturn) = SPReturn(Int(x))
SPMode(x::SPMode) = SPMode(Int(x))
SPEvent(x::SPEvent) = SPEvent(Int(x))
SPBuffer(x::SPBuffer) = SPBuffer(Int(x))
SPParity(x::SPParity) = SPParity(Int(x))
SPrts(x::SPrts) = SPrts(Int(x))
SPcts(x::SPcts) = SPcts(Int(x))
SPdtr(x::SPdtr) = SPdtr(Int(x))
SPdsr(x::SPdsr) = SPdsr(Int(x))
SPXonXoff(x::SPXonXoff) = SPXonXoff(Int(x))
SPFlowControl(x::SPFlowControl) = SPFlowControl(Int(x))
SPSignal(x::SPSignal) = SPSignal(Int(x))
SPTransport(x::SPTransport) = SPTransport(Int(x))


function check(ret::SPReturn)
    ret >= SP_OK && return Int(ret)

    msg = "libserialport returned $ret - "

    if ret == SP_ERR_ARG
        msg *= "Function was called with invalid arguments."
    elseif ret == SP_ERR_FAIL
        # Note: these functions only return valid info after SP_ERR_FAIL
        # Don't use them elsewhere
        println("OS error code $(sp_last_error_code()): $(sp_last_error_message())")
        msg *= "Host OS reported a failure."
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
    portp = PortP()
    check(ccall(
        (:sp_get_port_by_name, libserialport),
        SPReturn,
        (Ptr{UInt8}, PortP),
        portname,
        portp,
    ))
    portp[]
end

# void sp_free_port(struct sp_port *port);
function sp_free_port(port::Port)
    ccall((:sp_free_port, libserialport), Nothing, (Port,), port)
end

# enum sp_return sp_list_ports(struct sp_port ***list_ptr);
function sp_list_ports()
    ports = Ref{Ptr{Ptr{SPPort}}}()
    check(ccall((:sp_list_ports, libserialport), SPReturn, (Ref{Ptr{Ptr{SPPort}}},), ports))
    return ports[]
end

# enum sp_return sp_copy_port(const struct sp_port *port, struct sp_port **copy_ptr);
function sp_copy_port(port::Port)
    port_copy = PortP()
    check(ccall((:sp_copy_port, libserialport), SPReturn, (Port, PortP), port, port_copy))
    return port_copy[]
end

# void sp_free_port_list(struct sp_port **ports);
function sp_free_port_list(ports::PortP)
    ccall((:sp_free_port_list, libserialport), Nothing, (PortP,), ports)
end

# enum sp_return sp_open(struct sp_port *port, enum sp_mode flags);
function sp_open(port::Port, mode::SPMode)
    check(ccall((:sp_open, libserialport), SPReturn, (Port, SPMode), port, mode))
end

# enum sp_return sp_close(struct sp_port *port);
function sp_close(port::Port)
    check(ccall((:sp_close, libserialport), SPReturn, (Port,), port))
end

# char *sp_get_port_name(const struct sp_port *port);
function sp_get_port_name(port::Port)
    cname = ccall((:sp_get_port_name, libserialport), Ptr{UInt8}, (Port,), port)
    name = cname != C_NULL ? unsafe_string(cname) : ""
end

# char *sp_get_port_description(const struct sp_port *port);
function sp_get_port_description(port::Port)
    d = ccall((:sp_get_port_description, libserialport), Ptr{UInt8}, (Port,), port)
    desc = d != C_NULL ? unsafe_string(d) : ""
end

# enum sp_transport sp_get_port_transport(const struct sp_port *port);
function sp_get_port_transport(port::Port)
    ccall((:sp_get_port_transport, libserialport), SPTransport, (Port,), port)
end

# enum sp_return sp_get_port_usb_bus_address(const struct sp_port *port, int *usb_bus, int *usb_address);
function sp_get_port_usb_bus_address(port::Port)

    if sp_get_port_transport(port) != SP_TRANSPORT_USB
        @warn "Port does not use USB transport"
        return
    end

    usb_bus = Ref{Cint}()
    usb_address = Ref{Cint}()
    ret = ccall(
        (:sp_get_port_usb_bus_address, libserialport),
        SPReturn,
        (Port, Ref{Cint}, Ref{Cint}),
        port,
        usb_bus,
        usb_address,
    )

    if ret == SP_ERR_SUPP
        return -1, -1
    end

    check(ret)
    return usb_bus[], usb_address[]
end

# enum sp_return sp_get_port_usb_vid_pid(const struct sp_port *port, int *usb_vid, int *usb_pid);
function sp_get_port_usb_vid_pid(port::Port)

    if sp_get_port_transport(port) != SP_TRANSPORT_USB
        @warn "Port does not use USB transport"
        return
    end

    vid = Ref{Cint}()
    pid = Ref{Cint}()
    ret = ccall(
        (:sp_get_port_usb_vid_pid, libserialport),
        SPReturn,
        (Port, Ref{Cint}, Ref{Cint}),
        port,
        vid,
        pid,
    )

    if ret == SP_ERR_SUPP
        return -1, -1
    end

    check(ret)
    return vid[], pid[]
end

# char *sp_get_port_usb_manufacturer(const struct sp_port *port);
function sp_get_port_usb_manufacturer(port::Port)
    m = ccall((:sp_get_port_usb_manufacturer, libserialport), Ptr{UInt8}, (Port,), port)
    manufacturer = (m != C_NULL) ? unsafe_string(m) : ""
end

# char *sp_get_port_usb_product(const struct sp_port *port);
function sp_get_port_usb_product(port::Port)
    p = ccall((:sp_get_port_usb_product, libserialport), Ptr{UInt8}, (Port,), port)
    product = (p != C_NULL) ? unsafe_string(p) : ""
end

# char *sp_get_port_usb_serial(const struct sp_port *port);
function sp_get_port_usb_serial(port::Port)
    s = ccall((:sp_get_port_usb_serial, libserialport), Ptr{UInt8}, (Port,), port)
    serial = (s != C_NULL) ? unsafe_string(s) : ""
end

# char *sp_get_port_bluetooth_address(const struct sp_port *port);
function sp_get_port_bluetooth_address(port::Port)
    a = ccall((:sp_get_port_bluetooth_address, libserialport), Ptr{UInt8}, (Port,), port)
    address = (a != C_NULL) ? unsafe_string(a) : ""
end

if Sys.iswindows()
    # TODO: on Windows, result should be Ref{HANDLE}
    sp_get_port_handle(port::Port) = error("Returning port handle not supported on Windows")
else
    # enum sp_return sp_get_port_handle(const struct sp_port *port, void *result_ptr);
    function sp_get_port_handle(port::Port)
        # For Linux and OS X
        result = Ref{Cint}(0)
        check(ccall(
            (:sp_get_port_handle, libserialport),
            SPReturn,
            (Port, Ref{Cint}),
            port,
            result,
        ))
        result[]
    end
end

# enum sp_return sp_new_config(struct sp_port_config **config_ptr);
function sp_new_config()
    pc = ConfigP()
    check(ccall((:sp_new_config, libserialport), SPReturn, (ConfigP,), pc))
    pc[]
end

# void sp_free_config(struct sp_port_config *config);
function sp_free_config(config::Config)
    ccall((:sp_free_config, libserialport), Nothing, (Config,), config)
end

function sp_get_config(port::Port)
    config = sp_new_config()
    check(ccall((:sp_get_config, libserialport), SPReturn, (Port, Config), port, config))
    config
end

# enum sp_return sp_set_config(struct sp_port *port, const struct sp_port_config *config);
function sp_set_config(port::Port, config::Config)
    check(ccall((:sp_set_config, libserialport), SPReturn, (Port, Config), port, config))
end

# enum sp_return sp_set_baudrate(struct sp_port *port, int baudrate);
function sp_set_baudrate(port::Port, baudrate::Integer)
    check(ccall((:sp_set_baudrate, libserialport), SPReturn, (Port, Cint), port, baudrate))
end

# enum sp_return sp_get_config_baudrate(const struct sp_port_config *config, int *baudrate_ptr);
function sp_get_config_baudrate(config::Config)
    baudrate = Ref{Cint}()
    check(ccall(
        (:sp_get_config_baudrate, libserialport),
        SPReturn,
        (Config, Ref{Cint}),
        config,
        baudrate,
    ))
    baudrate[]
end

# enum sp_return sp_set_config_baudrate(struct sp_port_config *config, int baudrate);
function sp_set_config_baudrate(config::Config, baudrate::Integer)
    check(ccall(
        (:sp_set_config_baudrate, libserialport),
        SPReturn,
        (Config, Cint),
        config,
        baudrate,
    ))
end

# enum sp_return sp_set_bits(struct sp_port *port, int bits);
function sp_set_bits(port::Port, bits::Integer)
    if !(5 <= bits <= 8)
        throw(ArgumentError(
            "libserialport allows 5-8 data bits/frame. Received $bits."))
    end
    check(ccall((:sp_set_bits, libserialport), SPReturn, (Port, Cint), port, bits))
end

# enum sp_return sp_get_config_bits(const struct sp_port_config *config, int *bits_ptr);
function sp_get_config_bits(config::Config)
    bits = Ref{Cint}()
    check(ccall(
        (:sp_get_config_bits, libserialport),
        SPReturn,
        (Config, Ref{Cint}),
        config,
        bits,
    ))
    bits[]
end

# enum sp_return sp_set_config_bits(struct sp_port_config *config, int bits);
function sp_set_config_bits(config::Config, bits::Integer)
    check(ccall(
        (:sp_set_config_bits, libserialport),
        SPReturn,
        (Config, Cint),
        config,
        bits,
    ))
end

# enum sp_return sp_set_parity(struct sp_port *port, enum sp_parity parity);
function sp_set_parity(port::Port, parity::SPParity)
    check(ccall((:sp_set_parity, libserialport), SPReturn, (Port, SPParity), port, parity))
end

# enum sp_return sp_get_config_parity(const struct sp_port_config *config, enum sp_parity *parity_ptr);
function sp_get_config_parity(config::Config)
    parity = Ref{SPParity}()
    check(ccall(
        (:sp_get_config_parity, libserialport),
        SPReturn,
        (Config, Ref{SPParity}),
        config,
        parity,
    ))
    parity[]
end

# enum sp_return sp_set_config_parity(struct sp_port_config *config, enum sp_parity parity);
function sp_set_config_parity(config::Config, parity::SPParity)
    check(ccall(
        (:sp_set_config_parity, libserialport),
        SPReturn,
        (Config, SPParity),
        config,
        parity,
    ))
end

# enum sp_return sp_set_stopbits(struct sp_port *port, int stopbits);
function sp_set_stopbits(port::Port, stopbits::Integer)
    check(ccall((:sp_set_stopbits, libserialport), SPReturn, (Port, Cint), port, stopbits))
end

# enum sp_return sp_get_config_stopbits(const struct sp_port_config *config, int *stopbits_ptr);
function sp_get_config_stopbits(config::Config)
    bits = Ref{Cint}()
    check(ccall(
        (:sp_get_config_stopbits, libserialport),
        SPReturn,
        (Config, Ref{Cint}),
        config,
        bits,
    ))
    bits[]
end

# enum sp_return sp_set_config_stopbits(struct sp_port_config *config, int stopbits);
function sp_set_config_stopbits(config::Config, stopbits::Integer)
    check(ccall(
        (:sp_set_config_stopbits, libserialport),
        SPReturn,
        (Config, Cint),
        config,
        stopbits,
    ))
end

# enum sp_return sp_set_rts(struct sp_port *port, enum sp_rts rts);
function sp_set_rts(port::Port, rts::SPrts)
    check(ccall((:sp_set_rts, libserialport), SPReturn, (Port, SPrts), port, rts))
end

# enum sp_return sp_get_config_rts(const struct sp_port_config *config, enum sp_rts *rts_ptr);
function sp_get_config_rts(config::Config)
    rts = Ref{SPrts}()
    check(ccall(
        (:sp_get_config_rts, libserialport),
        SPReturn,
        (Config, Ref{SPrts}),
        config,
        rts,
    ))
    rts[]
end

# enum sp_return sp_set_config_rts(struct sp_port_config *config, enum sp_rts rts);
function sp_set_config_rts(config::Config, rts::SPrts)
    check(ccall(
        (:sp_set_config_rts, libserialport),
        SPReturn,
        (Config, SPrts),
        config,
        rts,
    ))
end

# enum sp_return sp_set_cts(struct sp_port *port, enum sp_cts cts);
function sp_set_cts(port::Port, cts::SPcts)
    check(ccall((:sp_set_cts, libserialport), SPReturn, (Port, SPcts), port, cts))
end

# enum sp_return sp_get_config_cts(const struct sp_port_config *config, enum sp_cts *cts_ptr);
function sp_get_config_cts(config::Config)
    cts = Ref{SPcts}()
    check(ccall(
        (:sp_get_config_cts, libserialport),
        SPReturn,
        (Config, Ref{SPcts}),
        config,
        cts,
    ))
    cts[]
end

# enum sp_return sp_set_config_cts(struct sp_port_config *config, enum sp_cts cts);
function sp_set_config_cts(config::Config, cts::SPcts)
    check(ccall(
        (:sp_set_config_cts, libserialport),
        SPReturn,
        (Config, SPcts),
        config,
        cts,
    ))
end

# enum sp_return sp_set_dtr(struct sp_port *port, enum sp_dtr dtr);
function sp_set_dtr(port::Port, dtr::SPdtr)
    check(ccall((:sp_set_dtr, libserialport), SPReturn, (Port, SPdtr), port, dtr))
end

# enum sp_return sp_get_config_dtr(const struct sp_port_config *config, enum sp_dtr *dtr_ptr);
function sp_get_config_dtr(config::Config)
    dtr = Ref{SPdtr}()
    check(ccall(
        (:sp_get_config_dtr, libserialport),
        SPReturn,
        (Config, Ref{SPdtr}),
        config,
        dtr,
    ))
    dtr[]
end

# enum sp_return sp_set_config_dtr(struct sp_port_config *config, enum sp_dtr dtr);
function sp_set_config_dtr(config::Config, dtr::SPdtr)
    check(ccall(
        (:sp_set_config_dtr, libserialport),
        SPReturn,
        (Config, SPdtr),
        config,
        dtr,
    ))
end

# enum sp_return sp_set_dsr(struct sp_port *port, enum sp_dsr dsr);
function sp_set_dsr(port::Port, dsr::SPdsr)
    check(ccall((:sp_set_dsr, libserialport), SPReturn, (Port, SPdsr), port, dsr))
end

# enum sp_return sp_get_config_dsr(const struct sp_port_config *config, enum sp_dsr *dsr_ptr);
function sp_get_config_dsr(config::Config)
    dsr = Ref{SPdsr}()
    check(ccall(
        (:sp_get_config_dsr, libserialport),
        SPReturn,
        (Config, Ref{SPdsr}),
        config,
        dsr,
    ))
    dsr[]
end

# enum sp_return sp_set_config_dsr(struct sp_port_config *config, enum sp_dsr dsr);
function sp_set_config_dsr(config::Config, dsr::SPdsr)
    check(ccall(
        (:sp_set_config_dsr, libserialport),
        SPReturn,
        (Config, SPdsr),
        config,
        dsr,
    ))
end

# enum sp_return sp_set_xon_xoff(struct sp_port *port, enum sp_xonxoff xon_xoff);
function sp_set_xon_xoff(port::Port, xon_xoff::SPXonXoff)
    check(ccall(
        (:sp_set_xon_xoff, libserialport),
        SPReturn,
        (Port, SPXonXoff),
        port,
        xon_xoff,
    ))
end

# enum sp_return sp_get_config_xon_xoff(const struct sp_port_config *config, enum sp_xonxoff *xon_xoff_ptr);
function sp_get_config_xon_xoff(config::Config)
    xon_xoff = Ref{SPXonXoff}()
    check(ccall(
        (:sp_get_config_xon_xoff, libserialport),
        SPReturn,
        (Config, Ref{SPXonXoff}),
        config,
        xon_xoff,
    ))
    xon_xoff[]
end

# enum sp_return sp_set_config_xon_xoff(struct sp_port_config *config, enum sp_xonxoff xon_xoff);
function sp_set_config_xon_xoff(config::Config, xon_xoff::SPXonXoff)
    check(ccall(
        (:sp_set_config_xon_xoff, libserialport),
        SPReturn,
        (Config, SPXonXoff),
        config,
        SPXonXoff(xon_xoff),
    ))
end

# enum sp_return sp_set_config_flowcontrol(struct sp_port_config *config, enum sp_flowcontrol flowcontrol);
function sp_set_config_flowcontrol(config::Config, flowcontrol::SPFlowControl)
    check(ccall(
        (:sp_set_config_flowcontrol, libserialport),
        SPReturn,
        (Config, SPFlowControl),
        config,
        flowcontrol,
    ))
end

# enum sp_return sp_set_flowcontrol(struct sp_port *port, enum sp_flowcontrol flowcontrol);
function sp_set_flowcontrol(port::Port, flowcontrol::SPFlowControl)
    check(ccall(
        (:sp_set_flowcontrol, libserialport),
        SPReturn,
        (Port, SPFlowControl),
        port,
        flowcontrol,
    ))
end

# enum sp_return sp_blocking_read(struct sp_port *port, void *buf, size_t count, unsigned int timeout_ms);
function sp_blocking_read(
    port::Port,
    buffer::Union{Ref{T},Ptr{T}},
    nbytes::Integer,
    timeout_ms::Integer,
) where {T}
    # If the read succeeds, the return value is the number of bytes read.
    ret = check(ccall(
        (:sp_blocking_read, libserialport),
        SPReturn,
        (Port, Ptr{UInt8}, Csize_t, Cuint),
        port,
        buffer,
        sizeof(T) * nbytes,
        timeout_ms,
    ))
    return ret
end
function sp_blocking_read(port::Port, nbytes::Integer, timeout_ms::Integer)
    buffer = zeros(UInt8, nbytes)

    # If the read succeeds, the return value is the number of bytes read.
    ret = check(ccall(
        (:sp_blocking_read, libserialport),
        SPReturn,
        (Port, Ptr{UInt8}, Csize_t, Cuint),
        port,
        buffer,
        nbytes,
        timeout_ms,
    ))
    return ret, buffer
end

# enum sp_return sp_blocking_read_next(struct sp_port *port, void *buf, size_t count, unsigned int timeout_ms);
function sp_blocking_read_next(port::Port, nbytes::Integer, timeout_ms::Integer)
    buffer = zeros(UInt8, nbytes)

    # If the read succeeds, the return value is the number of bytes read.
    ret = check(ccall(
        (:sp_blocking_read_next, libserialport),
        SPReturn,
        (Port, Ptr{UInt8}, Csize_t, Cuint),
        port,
        buffer,
        nbytes,
        timeout_ms,
    ))
    return ret, buffer
end

# enum sp_return sp_nonblocking_read(struct sp_port *port, void *buf, size_t count);
function sp_nonblocking_read(
    port::Port,
    buffer::Union{Ref{T},Ptr{T}},
    nbytes::Integer,
) where {T}
    check(ccall(
        (:sp_nonblocking_read, libserialport),
        SPReturn,
        (Port, Ptr{UInt8}, Csize_t),
        port,
        buffer,
        sizeof(T) * nbytes,
    ))
end
function sp_nonblocking_read(port::Port, nbytes::Integer)
    buffer = zeros(UInt8, nbytes)

    # If the read succeeds, the return value is the number of bytes read.
    ret = check(ccall(
        (:sp_nonblocking_read, libserialport),
        SPReturn,
        (Port, Ptr{UInt8}, Csize_t),
        port,
        buffer,
        Csize_t(nbytes),
    ))
    return ret, buffer
end

# enum sp_return sp_blocking_write(struct sp_port *port, const void *buf, size_t count, unsigned int timeout_ms);
function sp_blocking_write(port::Port, buffer::Array{UInt8}, timeout_ms::Integer)
    check(ccall(
        (:sp_blocking_write, libserialport),
        SPReturn,
        (Port, Ptr{UInt8}, Csize_t, Cuint),
        port,
        pointer(buffer),
        length(buffer),
        timeout_ms,
    ))
end

"""
    function sp_blocking_write(port::Port, buffer::Union{Ref{T},Ptr{T}},
                               n::Integer = 1; timeout_ms::Integer = 0) where T`

Write the `sizeof(T)*n` bytes starting from address `buffer` to the
specified serial port, blocking until complete.

Note that this function only ensures that the accepted bytes have been
written to the OS; they may be held in driver or hardware buffers and
not yet physically transmitted. To check whether all written bytes
have actually been transmitted, use the `sp_output_waiting()` function.
To wait until all written bytes have actually been transmitted, use
the `sp_drain()` function.

Wait up to `timeout_ms` milliseconds, where zero means to wait indefinitely.

Returns the number of bytes written on success, or raises an `ErrorException`.
If the number of bytes returned is less than that requested, the
timeout was reached before the requested number of bytes was written.
If `timeout_ms` is zero, the function will always return either the
requested number of bytes or raise an `ErrorException`. In the event of an
error there is no way to determine how many bytes were sent before the
error occured.
"""
function sp_blocking_write(
    port::Port,
    buffer::Union{Ref{T},Ptr{T}},
    n::Integer = 1,
    timeout_ms::Integer = 0,
) where {T}
    # If successful, the return value is the number of bytes written.
    ret = check(ccall(
        (:sp_blocking_write, libserialport),
        SPReturn,
        (Port, Ptr{UInt8}, Csize_t, Cuint),
        port,
        buffer,
        sizeof(T) * n,
        timeout_ms,
    ))
    return ret
end

function sp_blocking_write(port::Port, buffer::String, timeout_ms::Integer)
    # If successful, the return value is the number of bytes written.
    ret = check(ccall(
        (:sp_blocking_write, libserialport),
        SPReturn,
        (Port, Ptr{UInt8}, Csize_t, Cuint),
        port,
        buffer,
        length(buffer),
        timeout_ms,
    ))
    return ret
end

# enum sp_return sp_nonblocking_write(struct sp_port *port, const void *buf, size_t count);
function sp_nonblocking_write(port::Port, buffer::Array{UInt8})
    check(ccall(
        (:sp_nonblocking_write, libserialport),
        SPReturn,
        (Port, Ptr{UInt8}, Csize_t),
        port,
        pointer(buffer),
        length(buffer),
    ))
end

"""
    sp_nonblocking_write(port::Port, buffer::Union{Ptr{T},Ref{T}},
                         n::Integer = 1) where T`

Write the up to `sizeof(T)*n` bytes starting from address `buffer` to
the specified serial port, without blocking.

Note that this function only ensures that the accepted bytes have been
written to the OS; they may be held in driver or hardware buffers and
not yet physically transmitted. To check whether all written bytes
have actually been transmitted, use the `sp_output_waiting()`
function. To wait until all written bytes have actually been
transmitted, use the `sp_drain()` function.

Returns the number of bytes written on success, or raises an `ErrorException`.
The number of bytes returned may be any number from zero to the
maximum that was requested.
"""
function sp_nonblocking_write(
    port::Port,
    buffer::Union{Ptr{T},Ref{T}},
    n::Integer = 1,
) where {T}
    check(ccall(
        (:sp_nonblocking_write, libserialport),
        SPReturn,
        (Port, Ptr{UInt8}, Csize_t),
        port,
        buffer,
        n * sizeof(T),
    ))
end

function sp_nonblocking_write(port::Port, buffer::String)
    check(ccall(
        (:sp_nonblocking_write, libserialport),
        SPReturn,
        (Port, Ptr{UInt8}, Csize_t),
        port,
        buffer,
        sizeof(buffer),
    ))
end

# enum sp_return sp_input_waiting(struct sp_port *port);
"""
Returns the number of bytes in the input buffer.
"""
function sp_input_waiting(port::Port)
    check(ccall((:sp_input_waiting, libserialport), SPReturn, (Port,), port))
end

# enum sp_return sp_output_waiting(struct sp_port *port);
"""
Returns the number of bytes in the output buffer.
"""
function sp_output_waiting(port::Port)
    check(ccall((:sp_output_waiting, libserialport), SPReturn, (Port,), port))
end

# enum sp_return sp_flush(struct sp_port *port, enum sp_buffer buffers);
"""
    sp_flush(port::Port, buffers::SPBuffer)
    sp_flush(port::SerialPort, buffers::SPBuffer)

Discard data in the selected serial-port buffer(s).

Supported values for `buffers`: `SP_BUF_INPUT`, `SP_BUF_OUTPUT`, `SP_BUF_BOTH`

Returns `SP_OK` upon success or raises an `ErrorException` otherwise.

!!! note

    Not to be confused with `Base.flush`, which writes out buffered
    data rather than discarding it: the underlying `libserialport` C
    library unfortunately uses the verb “flush” differently from its
    normal meaning for `Base.IO` (`sp_drain` provides the latter
    in this library).
"""
function sp_flush(port::Port, buffers::SPBuffer)
    check(ccall((:sp_flush, libserialport), SPReturn, (Port, SPBuffer), port, buffers))
end

# enum sp_return sp_drain(struct sp_port *port);
"""
    sp_drain(port::Port)
    sp_drain(SerialPort::Port)

Wait for buffered data to be transmitted.
"""
function sp_drain(port::Port)
    check(ccall((:sp_drain, libserialport), SPReturn, (Port,), port))
end

# enum sp_return sp_new_event_set(struct sp_event_set **result_ptr);
function sp_new_event_set()
    event_set = Ref{Ptr{SPEventSet}}()
    check(ccall(
        (:sp_new_event_set, libserialport),
        SPReturn,
        (Ref{Ptr{SPEventSet}},),
        event_set,
    ))
    event_set[]
end

# enum sp_return sp_add_port_events(struct sp_event_set *event_set, const struct sp_port *port, enum sp_event mask);
function sp_add_port_events(event_set::Ref{SPEventSet}, port::Port, mask::SPEvent)
    check(ccall(
        (:sp_add_port_events, libserialport),
        SPReturn,
        (Ref{SPEventSet}, Port, SPEvent),
        event_set,
        port,
        mask,
    ))
end

# enum sp_return sp_wait(struct sp_event_set *event_set, unsigned int timeout_ms);
function sp_wait(event_set::Ref{SPEventSet}, timeout_ms::Integer)
    check(ccall(
        (:sp_wait, libserialport),
        SPReturn,
        (Ref{SPEventSet}, Cuint),
        event_set,
        timeout_ms,
    ))
end

# void sp_free_event_set(struct sp_event_set *event_set);
function sp_free_event_set(event_set::Ref{SPEventSet})
    check(ccall(
        (:sp_free_event_set, libserialport),
        SPReturn,
        (Ref{SPEventSet},),
        event_set,
    ))
end

# enum sp_return sp_get_signals(struct sp_port *port, enum sp_signal *signal_mask);
function sp_get_signals(port::Port, signal_mask::Ref{SPSignal})
    check(ccall(
        (:sp_get_signals, libserialport),
        SPReturn,
        (Port, Ref{SPSignal}),
        port,
        signal_mask,
    ))
    signal_mask[]
end

# enum sp_return sp_start_break(struct sp_port *port);
function sp_start_break(port::Port)
    check(ccall((:sp_start_break, libserialport), SPReturn, (Port,), port))
end

# enum sp_return sp_end_break(struct sp_port *port);
function sp_end_break(port::Port)
    check(ccall((:sp_end_break, libserialport), SPReturn, (Port,), port))
end

# int sp_last_error_code(void);
function sp_last_error_code()
    ccall((:sp_last_error_code, libserialport), Cint, ())
end

# char *sp_last_error_message(void);
function sp_last_error_message()
    msg = ccall((:sp_last_error_message, libserialport), Ptr{UInt8}, ())
    msg_jl = unsafe_string(msg)
    _sp_free_error_message(msg)
    return msg_jl
end

# void sp_free_error_message(char *message);
function _sp_free_error_message(message::Ptr{UInt8})
    ccall((:sp_free_error_message, libserialport), Nothing, (Ptr{UInt8},), message)
end

# Due to ccall's incomplete variadic argument support, the following two
# functions are not (yet) wrapped.
# void sp_set_debug_handler(void (*handler)(const char *format, ...));
# void sp_default_debug_handler(const char *format, ...);

# int sp_get_major_package_version(void);
function sp_get_major_package_version()
    ccall((:sp_get_major_package_version, libserialport), Cint, ())
end

# int sp_get_minor_package_version(void);
function sp_get_minor_package_version()
    ccall((:sp_get_minor_package_version, libserialport), Cint, ())
end

# int sp_get_micro_package_version(void);
function sp_get_micro_package_version()
    ccall((:sp_get_micro_package_version, libserialport), Cint, ())
end

# const char *sp_get_package_version_string(void);
function sp_get_package_version_string()
    ver = ccall((:sp_get_package_version_string, libserialport), Ptr{UInt8}, ())
    unsafe_string(ver)
end

# int sp_get_current_lib_version(void);
function sp_get_current_lib_version()
    ccall((:sp_get_current_lib_version, libserialport), Cint, ())
end

# int sp_get_revision_lib_version(void);
function sp_get_revision_lib_version()
    ccall((:sp_get_revision_lib_version, libserialport), Cint, ())
end

# int sp_get_age_lib_version(void);
function sp_get_age_lib_version()
    ccall((:sp_get_age_lib_version, libserialport), Cint, ())
end

# const char *sp_get_lib_version_string(void);
function sp_get_lib_version_string()
    ver = ccall((:sp_get_lib_version_string, libserialport), Ptr{UInt8}, ())
    unsafe_string(ver)
end

end # module wrap
