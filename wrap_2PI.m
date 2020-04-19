function result = wrap_2PI(rad)
    M_2PI = 2*pi();
    res = rem(rad, M_2PI);
    if res < 0
        res = res + M_2PI;
    end
    result = res;