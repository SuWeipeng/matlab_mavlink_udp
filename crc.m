function [crc] = crc(msg,crc_extra,extra)
% crc.m
% example:
% crc = dec2hex(crc(msg,1,54)
% ck_a = dec2hex(bitand(crc(msg,1,54),hex2dec('ff')))
% ck_b = dec2hex(bitand(bitshift(crc(msg,1,54),-8),hex2dec('ff')))
crc = hex2dec('ffff');
for i = [2:size(msg,1)-2]
    tmp = bitand(bitxor(hex2dec(msg(i,:)),bitand(crc,hex2dec('ff'))),255);
    tmp = bitand(bitxor(tmp,bitshift(tmp,4)),255);
    tmp_1 = bitand(bitxor(bitshift(crc,-8),bitshift(tmp,8)),65535);
    tmp_2 = bitand(bitxor(tmp_1,bitshift(tmp,3)),65535);
    crc = bitand(bitxor(tmp_2,bitshift(tmp,-4)),65535);
end
if crc_extra == 1
    tmp = bitand(bitxor(extra,bitand(crc,hex2dec('ff'))),255);
    tmp = bitand(bitxor(tmp,bitshift(tmp,4)),255);
    tmp_1 = bitand(bitxor(bitshift(crc,-8),bitshift(tmp,8)),65535);
    tmp_2 = bitand(bitxor(tmp_1,bitshift(tmp,3)),65535);
end
crc = bitand(bitxor(tmp_2,bitshift(tmp,-4)),65535);
end
