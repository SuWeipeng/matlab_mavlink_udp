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
h1 = plot(time,ax,'-r.');
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
title("Ax",'FontSize',12,'FontWeight','bold','Color','r','Rotation',0)

subplot(3,2,3);
h2 = plot(time,ay,'-r.');
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
title("Ay",'FontSize',12,'FontWeight','bold','Color','r','Rotation',0)

subplot(3,2,5);
h3 = plot(time,az,'-r.');
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
title("Az",'FontSize',12,'FontWeight','bold','Color','r','Rotation',0)

subplot(3,2,[2 4 6])
h4 = surf(s_x, s_y, s_z);
grid on
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
 
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
                        ax_plot   = ax;
                        ay_plot   = ay;
                        az_plot   = az;
                    else
                        time_plot = time(size(time,1)-100:end);
                        ax_plot   = ax(size(ax,1)-100:end);
                        ay_plot   = ay(size(ay,1)-100:end);
                        az_plot   = az(size(az,1)-100:end);
                    end

                    h1.XData = time_plot;
                    h1.YData = ax_plot;
                    h2.XData = time_plot;
                    h2.YData = ay_plot;
                    h3.XData = time_plot;
                    h3.YData = az_plot;
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
