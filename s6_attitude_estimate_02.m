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

phi = 0;
theta = 0;

points = RY(RX([point_a,point_b,point_c],phi),theta);

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

attitude = [0, 0, 0]';
phi      = attitude(1,1);
theta    = attitude(2,1);
psi      = attitude(3,1);
i = 1;
while 1
    tic
    A = dec2hex(fread(t));
    if ~isempty(A)
        msg_id = uint8_t(A,6);
        switch msg_id
            case 15
                if int16(crc(A,1,144)) == get_crc(A)                    
                    if i<=100
                        time(i,1) = uint32_t(A,7,10);
                        ax(i,1) = single(int16_t(A,11,12) * 9.8) / 1000;
                        ay(i,1) = single(int16_t(A,13,14) * 9.8) / 1000;
                        az(i,1) = single(int16_t(A,15,16) * 9.8) / 1000;
                        gx(i,1) = double(int16_t(A,17,18)) * pi() / 18000;
                        gy(i,1) = double(int16_t(A,19,20)) * pi() / 18000;
                        gz(i,1) = double(int16_t(A,21,22)) * pi() / 18000;
                        
                        temp = gx(i,1);
                        gx(i,1) = -gy(i,1);
                        gy(i,1) = temp;
                        
                        gyro = [gx(i,1) gy(i,1) gz(i,1)]';
                        
                        trans = [1, sin(phi) * tan(theta),  cos(phi) * tan(theta);
                                 0, cos(phi)             , -sin(phi);
                                 0, sin(phi) * sec(theta),  cos(phi) * sec(theta)];
                             
                        if i > 1
                            dt = single(time(i,1) - time(i-1,1)) / 1000;
                            attitude = attitude + trans * gyro .* dt;
                        else
                            dt = 0;
                        end                        
                        
                        phi   = attitude(1,1);
                        theta = attitude(2,1);
                        psi   = attitude(3,1);
                        
                        point_a = RZ(RY(RX([1,0,0]',phi),theta),psi);
                        point_b = RZ(RY(RX([0,-1,0]',phi),theta),psi);
                        point_c = RZ(RY(RX([0,1,0]',phi),theta),psi);
                        
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
                            'phi: ', num2str(phi),', ' ...
                            'theta: ', num2str(theta)])
                    else
                        time(1,:) = [];
                        ax(1,:) = [];
                        ay(1,:) = [];
                        az(1,:) = [];
                        gx(1,:) = []; 
                        gy(1,:) = [];
                        gz(1,:) = [];
                        time(100,1) = uint32_t(A,7,10);
                        ax(100,1) = single(int16_t(A,11,12) * 9.8) / 1000;
                        ay(100,1) = single(int16_t(A,13,14) * 9.8) / 1000;
                        az(100,1) = single(int16_t(A,15,16) * 9.8) / 1000;
                        gx(100,1) = double(int16_t(A,17,18)) * pi() / 18000;
                        gy(100,1) = double(int16_t(A,19,20)) * pi() / 18000;
                        gz(100,1) = double(int16_t(A,21,22)) * pi() / 18000;
                        
                        temp = gx(100,1);
                        gx(100,1) = -gy(100,1);
                        gy(100,1) = temp;
                        
                        gyro = [gx(100,1) gy(100,1) gz(100,1)]';
                        
                        trans = [1, sin(phi) * tan(theta),  cos(phi) * tan(theta);
                                 0, cos(phi)             , -sin(phi);
                                 0, sin(phi) * sec(theta),  cos(phi) * sec(theta)];
                             
                        dt = single(time(100,1) - time(99,1)) / 1000;
                        attitude = attitude + trans * gyro .* dt;
                        
                        phi   = attitude(1,1);
                        theta = attitude(2,1);
                        psi   = attitude(3,1);
                        
                        point_a = RY(RX([1,0,0]',phi),theta);
                        point_b = RY(RX([0,-1,0]',phi),theta);
                        point_c = RY(RX([0,1,0]',phi),theta);
                        
                        point_d = point_b - 0.5 * (point_b - point_c);
                        s_x = [point_a(1,1), point_b(1,1), point_c(1,1), point_a(1,1);
                            point_a(1,1), point_c(1,1), point_d(1,1), point_a(1,1)];
                        s_y = [point_a(2,1), point_b(2,1), point_c(2,1), point_a(2,1);
                            point_a(2,1), point_c(2,1), point_d(2,1), point_a(2,1)];
                        s_z = [point_a(3,1), point_b(3,1), point_c(3,1), point_a(3,1);
                            point_a(3,1), point_c(3,1), point_d(3,1), point_a(3,1)];                        
                        
                        
                        disp(['time: ',num2str(time(100,1)),', ' ...
                            'gx: ',num2str(gx(100,1)),', ' ...
                            'gy: ', num2str(gy(100,1)),', ' ...
                            'gz: ', num2str(gz(100,1)),', ' ...
                            'phi: ', num2str(phi),', ' ...
                            'theta: ', num2str(theta)])
                    end

                    h1.XData = time;
                    h1.YData = ax;
                    h2.XData = time;
                    h2.YData = ay;
                    h3.XData = time;
                    h3.YData = az;
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
