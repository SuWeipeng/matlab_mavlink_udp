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

subplot(2,2,[1 2]);
h1 = plot(time,gx,'-r.');
hold on
grid on
h2 = plot(time,gx,'-b.');
ylim([-100 100])
set(gca, 'GridLineStyle', ':');
set(gca, 'GridAlpha', 1);
set(gca, 'YTick', -100:0.2:100);
N = 2;
a1 = get(gca,'YTickLabel');
b1 = cell(size(a1));
b1(mod(1:size(a1,1),N)==1,:) = a1(mod(1:size(a1,1),N)==1,:);
set(gca,'YTickLabel',b1);
ylim([-inf inf])
title("Roll",'FontSize',12,'FontWeight','bold','Color','r','Rotation',0)

subplot(2,2,[3 4]);
h3 = plot(time,gy,'-r.');
hold on
grid on
h4 = plot(time,gy,'-b.');
ylim([-100 100])
set(gca, 'GridLineStyle', ':');
set(gca, 'GridAlpha', 1);
set(gca, 'YTick', -100:0.2:100);
N = 2;
a2 = get(gca,'YTickLabel');
b2 = cell(size(a2));
b2(mod(1:size(a2,1),N)==1,:) = a2(mod(1:size(a2,1),N)==1,:);
set(gca,'YTickLabel',b2);
ylim([-inf inf])
title("Pitch",'FontSize',12,'FontWeight','bold','Color','r','Rotation',0)
 
t = udp('127.0.0.1', 80, 'LocalPort', 14550);
t.InputBufferSize = 1024*10;

fopen(t)

unit_g        = [0;0;1];
phi_dcm       = 0;
theta_dcm     = 0;
roll          = 0;
pitch         = 0;
alpha         = 0.3;

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
                    gx(i,1) = single(int16_t(A,17,18)) * pi() / 18000;
                    gy(i,1) = single(int16_t(A,19,20)) * pi() / 18000;
                    gz(i,1) = single(int16_t(A,21,22)) * pi() / 18000;
                    
                    temp = ax(i,1);
                    ax(i,1) = -ay(i,1);
                    ay(i,1) = -temp;
                    
                    temp = gx(i,1);
                    gx(i,1) = -gy(i,1);
                    gy(i,1) = -temp;
                    
                    gz(i,1) = -gz(i,1);
                    
                    acc_vector = [ax(i,1);ay(i,1);az(i,1)];
                    acc_norm   = acc_vector / norm(acc_vector);
                    roll(i,1)  = -asin(acc_norm(2));
                    pitch(i,1) = atan2(acc_norm(1),acc_norm(3));
                    
                    if i > 1
                        dt = single(time(i,1) - time(i-1,1)) / 1000;
                    
                        delta_roll  = gx(i,1)*dt;
                        delta_pitch = gy(i,1)*dt;
                        delta_yaw   = gz(i,1)*dt;
                        unit_g = RZ(RY(RX(unit_g,delta_roll),delta_pitch),delta_yaw);
                        unit_g = (1 - alpha) * unit_g + alpha * acc_norm; 
                    else
                        dt = 0;
                    end
                    
                    phi_dcm(i,1)    = -asin(unit_g(2));
                    theta_dcm(i,1)  = atan2(unit_g(1),unit_g(3));
                    
                    disp(['time: ',num2str(time(i,1)),', ' ...
                          'yaw: ', num2str(rad2deg(atan2(unit_g(2),unit_g(1))))])
                    if i<=100
                        time_plot  = time;
                        phi_plot   = roll;
                        theta_plot = pitch;
                        phi_dcm_plot   = phi_dcm;
                        theta_dcm_plot = theta_dcm;
                    else
                        time_plot  = time(size(time,1)-100:end);
                        phi_plot   = roll(size(roll,1)-100:end);
                        theta_plot = pitch(size(pitch,1)-100:end);
                        phi_dcm_plot   = phi_dcm(size(phi_dcm,1)-100:end);
                        theta_dcm_plot = theta_dcm(size(theta_dcm,1)-100:end);
                    end

                    h1.XData = time_plot;
                    h1.YData = phi_plot;
                    h2.XData = time_plot;
                    h2.YData = phi_dcm_plot;
                    h3.XData = time_plot;
                    h3.YData = theta_plot;
                    h4.XData = time_plot;
                    h4.YData = theta_dcm_plot;
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
