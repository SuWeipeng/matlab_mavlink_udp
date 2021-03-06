function [ val ] = int16_t(input, from, to)
% float.m
% decode from hex to float
% input - mavlink message from udp
% from  - start byte index
% to    - end byte index
if size([from:to],2) ~= 2
    disp('error: too few arguments')
elseif size(input,1) <= to
    disp('error: message error')
else
    for i = [1:2:4]
        tmp(1,i) = input(to,1);
        tmp(1,i+1) = input(to,2);
        to = to - 1;
    end
    val = typecast(uint16(hex2dec(tmp)),'int16');
end
end