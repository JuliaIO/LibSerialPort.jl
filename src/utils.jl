import Base: readline, readuntil

export readline, readuntil, eachline

function readline(s::IO, timeout::T; keep::Bool=false) where {T<:Real}
    line = readuntil(s, 0x0a, timeout, keep=true)
    i = length(line)
    if keep || i == 0 || line[i] != 0x0a
        return String(line)
    elseif i < 2 || line[i-1] != 0x0d
        return String(resize!(line,i-1))
    else
        return String(resize!(line,i-2))
    end
end
function readuntil(s::IO, delim::AbstractChar, timeout::T; keep::Bool=false) where {T<:Real}
    if delim ≤ '\x7f'
        return Base.readuntil_string(s, delim % UInt8, keep)
    end
    out = IOBuffer()
    t = Timer(timeout)
    while !eof(s) || !isopen(t)
        c = read(s, Char)
        if c == delim
            keep && write(out, c)
            break
        end
        write(out, c)
    end
    return String(take!(out))
end
function readuntil(s::IO, delim::T, timeout::U; keep::Bool=false) where {T, U<:Real}
    out = (T === UInt8 ? Base.StringVector(0) : Vector{T}())
    t = Timer(timeout)
    while !eof(s) || !isopen(t)
        c = read(s, T)
        if c == delim
            keep && push!(out, c)
            break
        end
        push!(out, c)
    end
    return out
end

function readlines(s::IO, timeout::T; kw...) where {T<:Real}
    collect(eachline(s, timeout; kw...))
end


struct EachLineSerial{IOT <: IO, T <: Real}
    stream::IOT
    timeout::T
    ondone::Function
    keep::Bool
    function EachLineSerial(stream::IO, timeout::T; ondone::Function=()->nothing, keep::Bool=false) where {T<:Real}
        new{typeof(stream), typeof(timeout)}(stream, timeout, ondone, keep)
    end
end

function eachline(stream::IO, timeout::T; keep::Bool=false) where {T<:Real}
    EachLineSerial(stream, timeout, keep=keep)::EachLineSerial
end

function iterate(itr::EachLineSerial, state=nothing)
    eof(itr.stream) && return (itr.ondone(); nothing)
    (readline(itr.stream, itr.timeout, keep=itr.keep), nothing)
end

eltype(::Type{<:EachLineSerial}) = String

IteratorSize(::Type{<:EachLineSerial}) = SizeUnknown()
