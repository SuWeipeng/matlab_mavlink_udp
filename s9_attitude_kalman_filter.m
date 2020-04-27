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
trans = [1, 0, 0;
         0, 1, 0;
         0, 0, 1];
     
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

var_driver = 1.5e-6;
var_init   = 1e-2;
var_acc    = [var_init, var_init]';

dt = 0;
A = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
B = [dt 0 0 0; 0 0 dt 0]';
C = [1 0 0 0; 0 0 1 0];
P = eye(4);
Q = eye(4) * 3e-3;
R = eye(2);
R(1,1) = R(1,1) * var_acc(1);
R(2,2) = R(2,2) * var_acc(2);
state_estimate = [0 0 0 0]';

alpha = 0.1;
attitude_gyro  = [0,0,0]';
attitude_acc   = [0,0,0]';
phi_flt    = state_estimate(1,1);
theta_flt  = state_estimate(3,1);
phi_acc    = attitude_acc(1,1);
theta_acc  = attitude_acc(2,1);
phi_gyro   = attitude_gyro(1,1);
theta_gyro = attitude_gyro(2,1);
psi_gyro   = attitude_gyro(3,1);

subplot(2,2,[1]);
h1 = plot(time,ax,'-r.');
hold on
grid on
h2 = plot(time,phi_flt,'LineWidth',3,'Color',[1 0 1]);
% h5 = plot(time,gx,'-b.');
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
title("phi",'FontSize',12,'FontWeight','bold','Color','r','Rotation',0)

subplot(2,2,[3]);
h3 = plot(time,ay,'-r.');
hold on
grid on
h4 = plot(time,theta_flt,'LineWidth',3,'Color',[1 0 1]);
% h6 = plot(time,gy,'-b.');
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
title("theta",'FontSize',12,'FontWeight','bold','Color','r','Rotation',0)
 
subplot(2,2,[2 4])
h7 = surf(s_x, s_y, s_z);
grid on
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])

t = udp('127.0.0.1', 80, 'LocalPort', 14550);
t.InputBufferSize = 1024*10;

fopen(t)

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
                    
                    temp = ax(i,1);
                    ax(i,1) = -ay(i,1);
                    ay(i,1) = temp;
                    
                    attitude_acc = [atan2( ay(i,1), sqrt(ax(i,1) ^ 2 + az(i,1) ^ 2));
                                    atan2(-ax(i,1), sqrt(ay(i,1) ^ 2 + az(i,1) ^ 2));
                                    0];
                                
                    gyro = [gx(i,1) gy(i,1) gz(i,1)]';
                    
                    if i > 1
                        dt = single(time(i,1) - time(i-1,1)) / 1000;
                        A  = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
                        B  = [dt 0 0 0; 0 0 dt 0]';
                        
                        trans = [1, sin(phi_flt(i-1,1)) * tan(theta_flt(i-1,1)),  cos(phi_flt(i-1,1)) * tan(theta_flt(i-1,1));
                                 0, cos(phi_flt(i-1,1))                        , -sin(phi_flt(i-1,1));
                                 0, sin(phi_flt(i-1,1)) * sec(theta_flt(i-1,1)),  cos(phi_flt(i-1,1)) * sec(theta_flt(i-1,1))];
                             
                        R = eye(2);
                        R(1,1) = R(1,1) * var_acc(1);
                        R(2,2) = R(2,2) * var_acc(2);
                        
                        measurement = [attitude_acc(1,1) attitude_acc(2,1)]';
                        attitude_angular_rate = trans * gyro;
                        attitude_gyro = attitude_gyro + trans * gyro .* dt;
                        
                        state_estimate = A * state_estimate + B * [attitude_angular_rate(1,1), attitude_angular_rate(2,1)]';
                        P = A * P * A' + Q;
                        K = P * C' * inv(R + C * P * C');
                        state_estimate = state_estimate + K * (measurement - C * state_estimate);
                        P = (eye(4) - K * C) * P;
                    end
                    
                    phi_gyro(i,1)   = attitude_gyro(1,1);
                    theta_gyro(i,1) = attitude_gyro(2,1);
                    psi_gyro(i,1)   = attitude_gyro(3,1);
                    
                    phi_acc(i,1)   = attitude_acc(1,1);
                    theta_acc(i,1) = attitude_acc(2,1);
                    
                    phi_flt(i,1) =  state_estimate(1,1);
                    theta_flt(i,1) = state_estimate(3,1);
                    
                    cnt = 50;
                    if i>cnt
                        var_acc = [var(phi_acc(size(phi_acc,1)-cnt:end)), var(theta_acc(size(theta_acc,1)-cnt:end))]';
                        
                        l_lim = 1e-3;
                        h_lim = 1.3e0;
                        if var_acc(1) < l_lim
                            var_acc(1) = l_lim;
                        end
                        if var_acc(2) < l_lim
                            var_acc(2) = l_lim;
                        end
                        
                        if var_acc(1) > h_lim
                            var_acc(1) = h_lim;
                        end
                        if var_acc(2) > h_lim
                            var_acc(2) = h_lim;
                        end
                    end
                    
                    if i<=100
                        time_plot       = time;
                        phi_acc_plot    = phi_acc;
                        phi_flt_plot    = phi_flt;
                        theta_acc_plot  = theta_acc;
                        theta_flt_plot  = theta_flt;
                        phi_gyro_plot   = phi_gyro;
                        theta_gyro_plot = theta_gyro;
                    else
                        time_plot       = time(size(time,1)-100:end);
                        phi_acc_plot    = phi_acc(size(phi_acc,1)-100:end);
                        phi_flt_plot    = phi_flt(size(phi_flt,1)-100:end);
                        theta_acc_plot  = theta_acc(size(theta_acc,1)-100:end);
                        theta_flt_plot  = theta_flt(size(theta_flt,1)-100:end);
                        phi_gyro_plot   = phi_gyro(size(phi_gyro,1)-100:end);
                        theta_gyro_plot = theta_gyro(size(theta_gyro,1)-100:end);
                    end
                    
                    point_a = RY(RX([1,0,0]' ,phi_flt(i,1)),theta_flt(i,1));
                    point_b = RY(RX([0,-1,0]',phi_flt(i,1)),theta_flt(i,1));
                    point_c = RY(RX([0,1,0]' ,phi_flt(i,1)),theta_flt(i,1));
                    
                    point_d = point_b - 0.5 * (point_b - point_c);
                    s_x = [point_a(1,1), point_b(1,1), point_c(1,1), point_a(1,1);
                           point_a(1,1), point_c(1,1), point_d(1,1), point_a(1,1)];
                    s_y = [point_a(2,1), point_b(2,1), point_c(2,1), point_a(2,1);
                           point_a(2,1), point_c(2,1), point_d(2,1), point_a(2,1)];
                    s_z = [point_a(3,1), point_b(3,1), point_c(3,1), point_a(3,1);
                           point_a(3,1), point_c(3,1), point_d(3,1), point_a(3,1)];
                       
                    disp(['var_phi: ',num2str(var_acc(1)),', ' ...
                        'var_theta: ', num2str(var_acc(2))])
                    
                    h1.XData = time_plot;
                    h1.YData = phi_acc_plot;
                    h2.XData = time_plot;
                    h2.YData = phi_flt_plot;
                    h3.XData = time_plot;
                    h3.YData = theta_acc_plot;
                    h4.XData = time_plot;
                    h4.YData = theta_flt_plot;
                    h5.XData = time_plot;
                    h5.YData = phi_gyro_plot;
                    h6.XData = time_plot;
                    h6.YData = theta_gyro_plot;    
                    h7.XData = s_x;
                    h7.YData = s_y;
                    h7.ZData = s_z;
                    title(sprintf('phi:%1.3f, theta:%1.3f',var_acc(1), var_acc(2)),'FontSize',26,'FontWeight','bold','Color','r','Rotation',0)
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