function [crc] = get_crc(msg)
to = size(msg,1);
for i = [1:2:4]
    tmp(1,i) = msg(to,1);
    tmp(1,i+1) = msg(to,2);
    to = to - 1;
end
crc = typecast(uint16(hex2dec(tmp)),'int16');
end
