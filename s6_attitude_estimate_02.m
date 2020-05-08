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

point_a = [1,0,0]';
point_b = [0,-1,0]';
point_c = [0,1,0]';
point_d = point_b - 0.5 * (point_b - point_c);
s_x = [point_a(1,1), point_b(1,1), point_c(1,1), point_a(1,1);
       point_a(1,1), point_c(1,1), point_d(1,1), point_a(1,1)];
s_y = [point_a(2,1), point_b(2,1), point_c(2,1), point_a(2,1);
       point_a(2,1), point_c(2,1), point_d(2,1), point_a(2,1)];
s_z = [point_a(3,1), point_b(3,1), point_c(3,1), point_a(3,1);
       point_a(3,1), point_c(3,1), point_d(3,1), point_a(3,1)];

subplot(3,2,1);
h1 = plot(time,gx,'-r.');
grid on
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
title("Gx",'FontSize',12,'FontWeight','bold','Color','r','Rotation',0)

subplot(3,2,3);
h2 = plot(time,gy,'-r.');
grid on
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
title("Gy",'FontSize',12,'FontWeight','bold','Color','r','Rotation',0)

subplot(3,2,5);
h3 = plot(time,gz,'-r.');
grid on
ylim([-100 100])
set(gca, 'GridLineStyle', ':');
set(gca, 'GridAlpha', 1);
set(gca, 'YTick', -100:0.2:100);
N = 2;
a3 = get(gca,'YTickLabel');
b3 = cell(size(a3));
b3(mod(1:size(a3,1),N)==1,:) = a3(mod(1:size(a3,1),N)==1,:);
set(gca,'YTickLabel',b3);
ylim([-inf inf])
title("Gz",'FontSize',12,'FontWeight','bold','Color','r','Rotation',0)

subplot(3,2,[2 4 6])
h4 = surf(s_x, s_y, s_z);
grid on
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
 
t = udp('127.0.0.1', 80, 'LocalPort', 14550);
t.InputBufferSize = 1024*10;

fopen(t)

attitude_gyro = [0, 0, 0]';
phi_gyro      = attitude_gyro(1,1);
theta_gyro    = attitude_gyro(2,1);
psi_gyro      = attitude_gyro(3,1);

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
                    
                    temp = gx(i,1);
                    gx(i,1) = -gy(i,1);
                    gy(i,1) = temp;
                    
                    gyro = [gx(i,1) gy(i,1) gz(i,1)]';
                    
                    if i > 1
                        trans = [1, sin(phi_gyro(i-1,1)) * tan(theta_gyro(i-1,1)),  cos(phi_gyro(i-1,1)) * tan(theta_gyro(i-1,1));
                                 0, cos(phi_gyro(i-1,1))                         , -sin(phi_gyro(i-1,1));
                                 0, sin(phi_gyro(i-1,1)) * sec(theta_gyro(i-1,1)),  cos(phi_gyro(i-1,1)) * sec(theta_gyro(i-1,1))];
                    else
                        trans = [1, 0, 0;
                                 0, 1, 0;
                                 0, 0, 1];
                    end
                    
                    if i > 1
                        dt = single(time(i,1) - time(i-1,1)) / 1000;
                        attitude_gyro = attitude_gyro + trans * gyro .* dt;
                    else
                        dt = 0;
                    end
                    
                    phi_gyro(i,1)   = attitude_gyro(1,1);
                    theta_gyro(i,1) = attitude_gyro(2,1);
                    psi_gyro(i,1)   = attitude_gyro(3,1);
                    
                    point_a = RZ(RY(RX([1,0,0]',phi_gyro(i,1)),theta_gyro(i,1)),psi_gyro(i,1));
                    point_b = RZ(RY(RX([0,-1,0]',phi_gyro(i,1)),theta_gyro(i,1)),psi_gyro(i,1));
                    point_c = RZ(RY(RX([0,1,0]',phi_gyro(i,1)),theta_gyro(i,1)),psi_gyro(i,1));
                    
                    point_d = point_b - 0.5 * (point_b - point_c);
                    s_x = [point_a(1,1), point_b(1,1), point_c(1,1), point_a(1,1);
                           point_a(1,1), point_c(1,1), point_d(1,1), point_a(1,1)];
                    s_y = [point_a(2,1), point_b(2,1), point_c(2,1), point_a(2,1);
                           point_a(2,1), point_c(2,1), point_d(2,1), point_a(2,1)];
                    s_z = [point_a(3,1), point_b(3,1), point_c(3,1), point_a(3,1);
                           point_a(3,1), point_c(3,1), point_d(3,1), point_a(3,1)];
                    
                    disp(['time: ',num2str(time(i,1)),', ' ...
                          'gx: ',num2str(gx(i,1)),', ' ...
                          'gy: ', num2str(gy(i,1)),', ' ...
                          'gz: ', num2str(gz(i,1)),', ' ...
                          'phi: ', num2str(phi_gyro(i,1)),', ' ...
                          'theta: ', num2str(theta_gyro(i,1))])
                    if i<=100
                        time_plot = time;
                        gx_plot   = gx;
                        gy_plot   = gy;
                        gz_plot   = gz;
                    else
                        time_plot = time(size(time,1)-100:end);
                        gx_plot   = gx(size(gx,1)-100:end);
                        gy_plot   = gy(size(gy,1)-100:end);
                        gz_plot   = gz(size(gz,1)-100:end);
                    end

                    h1.XData = time_plot;
                    h1.YData = gx_plot;
                    h2.XData = time_plot;
                    h2.YData = gy_plot;
                    h3.XData = time_plot;
                    h3.YData = gz_plot;
                    h4.XData = s_x;
                    h4.YData = s_y;
                    h4.ZData = s_z;
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
