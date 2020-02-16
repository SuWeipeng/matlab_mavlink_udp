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

pwm = 0;
rpm = 0;
h = plot(pwm,rpm,'r.');

i = 1;
while 1
    tic
    A = dec2hex(fread(t));
    if ~isempty(A)
        msg_id = uint8_t(A,6);
        switch msg_id
            case 12
                if int16(crc(A,1,165)) == get_crc(A)
                    pwm(i,1) = int16_t(A,11,12);
                    rpm(i,1) = float(A,7,10);
                    disp(['pwm: ',num2str(pwm(i,1)),', ' ...
                        'rpm: ', num2str(rpm(i,1))])
                    h.XData = pwm;
                    h.YData = rpm;
                    drawnow
                    i = i + 1;
                end
            case 13
                if int16(crc(A,1,196)) == get_crc(A)
                    if uint8_t(A,7) == 1
                        fclose(t)
                        break
                    end
                end
            otherwise
                fclose(t)
                break
        end
    end
    freq(i,1) = 1/toc;
end

fclose(t)