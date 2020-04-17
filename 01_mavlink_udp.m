clc

t = udp('127.0.0.1', 80, 'LocalPort', 14550);
t.InputBufferSize = 89120;

fopen(t)

for i = 1:100
    tic
    A = dec2hex(fread(t));
    if ~isempty(A)
        vel_x = float(A,7,10);
        vel_y = float(A,11,14);
        rad_z = float(A,15,18);
        disp(['vel_x: ',num2str(vel_x),', ' ...
              'vel_y: ', num2str(vel_y),', ' ...
              'rad_z: ', num2str(rad_z)])
    end
    disp(['freq: ',num2str(1/toc), ' Hz'])
end

fclose(t)