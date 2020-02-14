clc
if exist('pwm')
    clear pwm
end
if exist('rpm')
    clear rpm
end

t = udp('127.0.0.1', 80, 'LocalPort', 14550);
t.InputBufferSize = 89120;

fopen(t)

for i = 1:1000
    tic
    A = dec2hex(fread(t));
    if int16(crc(A,1,165)) == get_crc(A)
        if ~isempty(A)
            pwm(i,1) = int16_t(A,11,12);
            rpm(i,1) = float(A,7,10);
            disp(['pwm: ',num2str(pwm(i,1)),', ' ...
                'rpm: ', num2str(rpm(i,1))])
        end
    end
    freq(i,1) = 1/toc;
end

fclose(t)