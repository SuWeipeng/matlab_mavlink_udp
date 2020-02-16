function [ val ] = uint8_t(input, to)
% float.m
% decode from hex to float
% input - mavlink message from udp
% to    - end byte index
if size(input,1) <= to
    disp('error: message error')
else
    tmp(1,1) = input(to,1);
    tmp(1,2) = input(to,2);
    val = uint8(hex2dec(tmp));
end
end