import Base: readline, readuntil
export readline, readuntil

#==
Timeout versions of tbe base functions
==#

"""
    readline(s::IO, timeout::T; keep::Bool=false) where {T<:Real}

Like Base.readline, except times-out after `timeout` seconds.
"""
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

"""
    readuntil(s::IO, delim::AbstractChar, timeout::T; keep::Bool=false) where {T<:Real}
    readuntil(s::IO, delim::T, timeout::U; keep::Bool=false) where {T, U<:Real}

Like Base.readuntil, except times-out after `timeout` seconds.
"""
function readuntil(s::IO, delim::AbstractChar, timeout::T; keep::Bool=false) where {T<:Real}
    if delim ≤ '\x7f'
        return Base.readuntil_string(s, delim % UInt8, keep)
    end
    out = IOBuffer()
    t = Timer(timeout)
    while !eof(s) && isopen(t)
        c = read(s, Char)
        if c == delim
            keep && write(out, c)
            break
        elseif c != 0x00
            write(out, c)
        end
        yield()
    end
    return String(take!(out))
end
function readuntil(s::IO, delim::T, timeout::U; keep::Bool=false) where {T, U<:Real}
    out = (T === UInt8 ? Base.StringVector(0) : Vector{T}())
    t = Timer(timeout)
    while !eof(s) && isopen(t)
        c = read(s, T)
        if c == delim
            keep && push!(out, c)
            break
        elseif c != 0x00
            push!(out, c)
        end
        yield()
    end
    return out
end
