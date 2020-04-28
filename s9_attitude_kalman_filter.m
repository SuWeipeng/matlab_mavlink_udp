clc
clear
close all

var_acc_init   = 1e-2;
var_gyro_init  = 6.75e0;

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

var_acc        = [var_acc_init, var_acc_init]';
var_gyro       = [var_gyro_init, var_gyro_init]';

dt = 60/1000;
A = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
C = eye(4);
P = eye(4);
Q = eye(4);
Q(1,1) = Q(1,1) * var_acc(1);
Q(2,2) = Q(2,2) * var_gyro(1);
Q(3,3) = Q(3,3) * var_acc(2);
Q(4,4) = Q(4,4) * var_gyro(2);
R = eye(4);
R(1,1) = R(1,1) * var_acc(1);
R(2,2) = R(2,2) * var_gyro(1);
R(3,3) = R(3,3) * var_acc(2);
R(4,4) = R(4,4) * var_gyro(2);
state_estimate = [0 0 0 0]';

attitude_gyro  = [0,0,0]';
attitude_acc   = [0,0,0]';
phi_flt    = state_estimate(1,1);
theta_flt  = state_estimate(3,1);
phi_acc    = attitude_acc(1,1);
theta_acc  = attitude_acc(2,1);
phi_gyro   = attitude_gyro(1,1);
theta_gyro = attitude_gyro(2,1);
psi_gyro   = attitude_gyro(3,1);

subplot(2,2,[1 2]);
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

subplot(2,2,[3 4]);
h3 = plot(time,ay,'-r.');
hold on
grid on
h4 = plot(time,theta_flt,'-b.');
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
title("var-phi",'FontSize',12,'FontWeight','bold','Color','r','Rotation',0)
legend("var-acc","var-gyro")
 
% subplot(2,2,[2 4])
% h7 = surf(s_x, s_y, s_z);
% grid on
% xlim([-1 1])
% ylim([-1 1])
% zlim([-1 1])

t = udp('127.0.0.1', 80, 'LocalPort', 14550);
t.InputBufferSize = 1024*10;

fopen(t)

i = 1;
while 1
    tic
    mav_msg = dec2hex(fread(t));
    if ~isempty(mav_msg)
        msg_id = uint8_t(mav_msg,6);
        switch msg_id
            case 15
                if int16(crc(mav_msg,1,144)) == get_crc(mav_msg)            
                    time(i,1) = uint32_t(mav_msg,7,10);
                    ax(i,1) = single(int16_t(mav_msg,11,12) * 9.8) / 1000;
                    ay(i,1) = single(int16_t(mav_msg,13,14) * 9.8) / 1000;
                    az(i,1) = single(int16_t(mav_msg,15,16) * 9.8) / 1000;
                    gx(i,1) = single(int16_t(mav_msg,17,18)) * pi() / 18000;
                    gy(i,1) = single(int16_t(mav_msg,19,20)) * pi() / 18000;
                    gz(i,1) = single(int16_t(mav_msg,21,22)) * pi() / 18000;
                    
                    temp = gx(i,1);
                    gx(i,1) = -gy(i,1);
                    gy(i,1) = temp;
                    
                    temp = ax(i,1);
                    ax(i,1) = -ay(i,1);
                    ay(i,1) = temp;
                    
                    attitude_acc = [atan2( ay(i,1), sqrt(ax(i,1) ^ 2 + az(i,1) ^ 2));
                                    atan2(-ax(i,1), sqrt(ay(i,1) ^ 2 + az(i,1) ^ 2));
                                    0];
                    phi_acc(i,1)   = attitude_acc(1,1);
                    theta_acc(i,1) = attitude_acc(2,1);
                                
                    gyro = [gx(i,1) gy(i,1) gz(i,1)]';
                    
                    if i > 1                        
                        trans = [1, sin(phi_flt(i-1,1)) * tan(theta_flt(i-1,1)),  cos(phi_flt(i-1,1)) * tan(theta_flt(i-1,1));
                                 0, cos(phi_flt(i-1,1))                        , -sin(phi_flt(i-1,1));
                                 0, sin(phi_flt(i-1,1)) * sec(theta_flt(i-1,1)),  cos(phi_flt(i-1,1)) * sec(theta_flt(i-1,1))];
                             
                        attitude_angular_rate = trans * gyro;
                        attitude_gyro = attitude_gyro + attitude_angular_rate .* dt;
                        phi_gyro(i,1)   = attitude_gyro(1,1);
                        theta_gyro(i,1) = attitude_gyro(2,1);
                        psi_gyro(i,1)   = attitude_gyro(3,1);
                        
                        measurement = [attitude_acc(1,1) attitude_angular_rate(1,1) attitude_acc(2,1) attitude_angular_rate(2,1)]';
                        
                        phi_rate_gyro(i,1) = measurement(2,1);
                        theta_rate_gyro(i,1) = measurement(4,1);
                        
                        cnt = 10;
                        if i>cnt
                            var_acc(:,i)  = [var(phi_acc(size(phi_acc,1)-cnt:end)), var(theta_acc(size(theta_acc,1)-cnt:end))]';
                            var_gyro(:,i) = [var(phi_rate_gyro(size(phi_rate_gyro,1)-cnt:end)), var(theta_rate_gyro(size(theta_rate_gyro,1)-cnt:end))]';
                        else
                            var_acc(:,i)  = [var(phi_acc), var(theta_acc)]';
                            var_gyro(:,i) = [var(phi_rate_gyro), var(theta_rate_gyro)]';
                        end
                        
                        R = eye(4);
                        R(1,1) = R(1,1) * var_acc(1,i);
                        R(2,2) = R(2,2) * var_gyro(1,i);
                        R(3,3) = R(3,3) * var_acc(2,i);
                        R(4,4) = R(4,4) * var_gyro(2,i);
                        
                        state_estimate = A * state_estimate;
                        P = A * P * A' + Q;
                        K = P * C' * inv(R + C * P * C');
                        state_estimate = state_estimate + K * (measurement - C * state_estimate);
                        P = (eye(4) - K * C) * P;
                    end
                    
                    phi_flt(i,1) =  state_estimate(1,1);
                    phi_rate(i,1) = state_estimate(2,1);
                    theta_flt(i,1) = state_estimate(3,1);
                    theta_rate(i,1) = state_estimate(4,1);
                    
                    if i<=100
                        time_plot       = time;
                        phi_acc_plot    = phi_acc;
                        phi_flt_plot    = phi_flt;
                        theta_acc_plot  = theta_acc;
                        theta_flt_plot  = theta_flt;
                        phi_gyro_plot   = phi_gyro;
                        theta_gyro_plot = theta_gyro;
                        var_acc_plot    = var_acc(1,:)';
                        var_gyro_plot   = var_gyro(1,:)';
                    else
                        time_plot       = time(size(time,1)-100:end);
                        phi_acc_plot    = phi_acc(size(phi_acc,1)-100:end);
                        phi_flt_plot    = phi_flt(size(phi_flt,1)-100:end);
                        theta_acc_plot  = theta_acc(size(theta_acc,1)-100:end);
                        theta_flt_plot  = theta_flt(size(theta_flt,1)-100:end);
                        phi_gyro_plot   = phi_gyro(size(phi_gyro,1)-100:end);
                        theta_gyro_plot = theta_gyro(size(theta_gyro,1)-100:end);
                        var_acc_plot    = var_acc(1,size(var_acc,2)-100:end)';
                        var_gyro_plot   = var_gyro(1,size(var_gyro,2)-100:end)';
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
                       
                    disp(['var_phi: ',num2str(var_acc(1,i)),', ' ...
                        'var_theta: ', num2str(var_acc(2,i))])
                    
                    h1.XData = time_plot;
                    h1.YData = phi_acc_plot;
                    h2.XData = time_plot;
                    h2.YData = phi_flt_plot;
                    h3.XData = time_plot;
                    h3.YData = var_acc_plot;
                    h4.XData = time_plot;
                    h4.YData = var_gyro_plot;
                    h5.XData = time_plot;
                    h5.YData = phi_gyro_plot;
                    h6.XData = time_plot;
                    h6.YData = theta_gyro_plot;    
                    h7.XData = s_x;
                    h7.YData = s_y;
                    h7.ZData = s_z;
                    drawnow
                    i = i + 1;
                end
            case 13
                if int16(crc(mav_msg,1,196)) == get_crc(mav_msg)
                    if uint8_t(mav_msg,7) == 1
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
close all
figure(1)
plot(time,phi_rate,'r',time,phi_rate_gyro,'b')
grid on
legend("phi-rate","phi-rate-gyro")
figure(2)
plot(time,theta_rate,'r',time,theta_rate_gyro,'b')
grid on
legend("theta-rate","theta-rate-gyro")
figure(3)
plot(time,phi_flt,'r',time,phi_acc,'b')
grid on
legend("phi-flt","phi-acc")
figure(4)
plot(time,theta_flt,'r',time,theta_acc,'b')
grid on
legend("theta-flt","theta-acc")