function result = wrap_PI(rad)
    M_PI  = pi();
    M_2PI = 2*pi();
    res = wrap_2PI(rad);
    if res > M_PI
        res = res - M_2PI;
    end
    result = res;