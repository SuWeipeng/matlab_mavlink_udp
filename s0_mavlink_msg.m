clc

t = udp('127.0.0.1', 80, 'LocalPort', 14550);
t.InputBufferSize = 89120;

fopen(t)

for i = 1:100
    A = dec2hex(fread(t));
    if ~isempty(A)
        i
        A
        size(A)
    end
end

fclose(t)