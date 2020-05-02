clc
clear
close all

time = 0;
ax = 0;
ay = 0;
az = 0;
gx = 0;
gy = 0;
gz = 0;

subplot(2,1,1)
h1 = plot(time,ax,'-r');
hold on
grid on
h2 = plot(time,ax,'--b.');

subplot(2,1,2)
h3 = plot(time,ay,'-r');
hold on
grid on
h4 = plot(time,ay,'--b.');
 
t = udp('127.0.0.1', 80, 'LocalPort', 14550);
t.InputBufferSize = 1024*10;

fopen(t)

attitude_acc = [0, 0, 0]';
phi_acc      = attitude_acc(1,1);
theta_acc    = attitude_acc(2,1);

i = 1;
while 1
    tic
    A = dec2hex(fread(t));
    if ~isempty(A)
        msg_id = uint8_t(A,6);
        switch msg_id
            case 15
                if int16(crc(A,1,144)) == get_crc(A)      
                    time(i,1) = uint32_t(A,7,10);
                    ax(i,1) = single(int16_t(A,11,12) * 9.8) / 1000;
                    ay(i,1) = single(int16_t(A,13,14) * 9.8) / 1000;
                    az(i,1) = single(int16_t(A,15,16) * 9.8) / 1000;
                    gx(i,1) = single(int16_t(A,17,18)) / 100;
                    gy(i,1) = single(int16_t(A,19,20)) / 100;
                    gz(i,1) = single(int16_t(A,21,22)) / 100;
                    
                    temp = ax(i,1);
                    ax(i,1) = -ay(i,1);
                    ay(i,1) = temp;
                    
                    phi_acc(i,1)   = atan2( ay(i,1), sqrt(ax(i,1) ^ 2 + az(i,1) ^ 2));
                    theta_acc(i,1) = atan2(-ax(i,1), sqrt(ay(i,1) ^ 2 + az(i,1) ^ 2));
                    
                    acc_vector = [ax(i,1);ay(i,1);az(i,1)];
                    acc_norm = acc_vector / norm(acc_vector);
                    
                    roll(i,1)      = asin(acc_norm(2));
                    pitch(i,1)     = -atan2(acc_norm(1),acc_norm(3));
                    
                    point_a = RY(RX([1,0,0]',phi_acc(i,1)),theta_acc(i,1));
                    point_b = RY(RX([0,-1,0]',phi_acc(i,1)),theta_acc(i,1));
                    point_c = RY(RX([0,1,0]',phi_acc(i,1)),theta_acc(i,1));
                    
                    point_d = point_b - 0.5 * (point_b - point_c);
                    s_x = [point_a(1,1), point_b(1,1), point_c(1,1), point_a(1,1);
                           point_a(1,1), point_c(1,1), point_d(1,1), point_a(1,1)];
                    s_y = [point_a(2,1), point_b(2,1), point_c(2,1), point_a(2,1);
                           point_a(2,1), point_c(2,1), point_d(2,1), point_a(2,1)];
                    s_z = [point_a(3,1), point_b(3,1), point_c(3,1), point_a(3,1);
                           point_a(3,1), point_c(3,1), point_d(3,1), point_a(3,1)];
                    
                    disp(['time: ',num2str(time(i,1)),', ' ...
                        'ax: ',num2str(ax(i,1)),', ' ...
                        'ay: ', num2str(ay(i,1)),', ' ...
                        'az: ', num2str(az(i,1)),', ' ...
                        'phi: ', num2str(phi_acc(i,1)),', ' ...
                        'theta: ', num2str(theta_acc(i,1))])
                    if i<=100
                        time_plot = time;
                        phi_acc_plot     = phi_acc;
                        theta_acc_plot   = theta_acc;
                        roll_plot        = roll;
                        pitch_plot       = pitch;
                    else
                        time_plot        = time(size(time,1)-100:end);
                        phi_acc_plot     = phi_acc(size(phi_acc,1)-100:end);
                        theta_acc_plot   = theta_acc(size(theta_acc,1)-100:end);
                        roll_plot        = roll(size(roll,1)-100:end);
                        pitch_plot       = pitch(size(pitch,1)-100:end);
                    end

                    h1.XData = time_plot;
                    h1.YData = phi_acc_plot;
                    h2.XData = time_plot;
                    h2.YData = roll_plot;
                    h3.XData = time_plot;
                    h3.YData = theta_acc_plot;
                    h4.XData = time_plot;
                    h4.YData = pitch_plot;
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
                continue
        end
    end
    freq(i,1) = 1/toc;
end

fclose(t)
