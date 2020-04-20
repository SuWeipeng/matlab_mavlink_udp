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

alpha = 0.1;
attitude_gyro  = [0,0,0]';
attitude_acc   = [0,0,0]';
attitude   = [0, 0, 0]';
phi_flt    = attitude(1,1);
theta_flt  = attitude(2,1);
psi        = attitude(3,1);
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
                    if i<=100
                        time(i,1) = uint32_t(A,7,10);
                        ax(i,1) = single(int16_t(A,11,12) * 9.8) / 1000;
                        ay(i,1) = single(int16_t(A,13,14) * 9.8) / 1000;
                        az(i,1) = single(int16_t(A,15,16) * 9.8) / 1000;
                        gx(i,1) = single(int16_t(A,17,18)) * pi() / 180000;
                        gy(i,1) = single(int16_t(A,19,20)) * pi() / 180000;
                        gz(i,1) = single(int16_t(A,21,22)) * pi() / 180000;
                        
                        temp = gx(i,1);
                        gx(i,1) = -gy(i,1);
                        gy(i,1) = temp;
                        
                        temp = ax(i,1);
                        ax(i,1) = -ay(i,1);
                        ay(i,1) = temp;
                        
                        gyro = [gx(i,1) gy(i,1) gz(i,1)]';
                        
                        if i > 1
                            trans = [1, sin(phi_flt(i-1,1)) * tan(theta_flt(i-1,1)),  cos(phi_flt(i-1,1)) * tan(theta_flt(i-1,1));
                                     0, cos(phi_flt(i-1,1))                        , -sin(phi_flt(i-1,1));
                                     0, sin(phi_flt(i-1,1)) * sec(theta_flt(i-1,1)),  cos(phi_flt(i-1,1)) * sec(theta_flt(i-1,1))];
                        else
                            trans = [1, 0, 0;
                                     0, 1, 0;
                                     0, 0, 1];
                        end
                             
                        attitude_acc = [atan2( ay(i,1), sqrt(ax(i,1) ^ 2 + az(i,1) ^ 2));
                                        atan2(-ax(i,1), sqrt(ay(i,1) ^ 2 + az(i,1) ^ 2));
                                        0];
                        if i > 1
                            dt = single(time(i,1) - time(i-1,1)) / 1000;
                            attitude = (attitude + trans * gyro .* dt) .* (1 - alpha) + attitude_acc .* alpha;
                            attitude_gyro = attitude_gyro + trans * gyro .* dt;
                        else
                            dt = 0;
                        end                        
                        
                        phi_gyro(i,1)   = attitude_gyro(1,1);
                        theta_gyro(i,1) = attitude_gyro(2,1);
                        psi_gyro(i,1)   = attitude_gyro(3,1);
                        
                        phi_acc(i,1)   = attitude_acc(1,1);
                        theta_acc(i,1) = attitude_acc(2,1);
                        
                        phi_flt(i,1) =  attitude(1,1);
                        theta_flt(i,1) = attitude(2,1);
                        
                        disp(['time: ',num2str(time(i,1)),', ' ...
                            'gx: ',num2str(gx(i,1)),', ' ...
                            'gy: ', num2str(gy(i,1)),', ' ...
                            'gz: ', num2str(gz(i,1)),', ' ...
                            'ax: ', num2str(ax(i,1)),', ' ...
                            'ay: ', num2str(ay(i,1)),', ' ...
                            'az: ', num2str(az(i,1))])
                    else
                        time(1,:) = [];
                        ax(1,:) = [];
                        ay(1,:) = [];
                        az(1,:) = [];
                        gx(1,:) = []; 
                        gy(1,:) = [];
                        gz(1,:) = [];
                        phi_gyro(1,:) = [];
                        theta_gyro(1,:) = [];
                        phi_acc(1,:) = [];
                        theta_acc(1,:) = [];
                        phi_flt(1,:) = [];
                        theta_flt(1,:) = [];
                        time(100,1) = uint32_t(A,7,10);
                        ax(100,1) = single(int16_t(A,11,12) * 9.8) / 1000;
                        ay(100,1) = single(int16_t(A,13,14) * 9.8) / 1000;
                        az(100,1) = single(int16_t(A,15,16) * 9.8) / 1000;
                        gx(100,1) = single(int16_t(A,17,18)) * pi() / 180000;
                        gy(100,1) = single(int16_t(A,19,20)) * pi() / 180000;
                        gz(100,1) = single(int16_t(A,21,22)) * pi() / 180000;
                        
                        temp = gx(100,1);
                        gx(100,1) = -gy(100,1);
                        gy(100,1) = temp;
                        
                        temp = ax(100,1);
                        ax(100,1) = -ay(100,1);
                        ay(100,1) = temp;
                        
                        gyro = [gx(100,1) gy(100,1) gz(100,1)]' .* 6;
                        
                        trans = [1, sin(phi_flt(99,1)) * tan(theta_flt(99,1)),  cos(phi_flt(99,1)) * tan(theta_flt(99,1));
                                 0, cos(phi_flt(99,1))                       , -sin(phi_flt(99,1));
                                 0, sin(phi_flt(99,1)) * sec(theta_flt(99,1)),  cos(phi_flt(99,1)) * sec(theta_flt(99,1))];
                        
                        attitude_acc = [atan2( ay(100,1), sqrt(ax(100,1) ^ 2 + az(100,1) ^ 2));
                                        atan2(-ax(100,1), sqrt(ay(100,1) ^ 2 + az(100,1) ^ 2));
                                        0];
                        dt = single(time(100,1) - time(99,1)) / 1000;
                        attitude = (attitude + trans * gyro .* dt) .* (1 - alpha) + attitude_acc .* alpha;
                        attitude_gyro = attitude_gyro + trans * gyro .* dt;
                        
                        phi_gyro(100,1)   = attitude_gyro(1,1);
                        theta_gyro(100,1) = attitude_gyro(2,1);
                        psi_gyro(100,1)   = attitude_gyro(3,1);
                        
                        phi_acc(100,1)   = attitude_acc(1,1);
                        theta_acc(100,1) = attitude_acc(2,1);
                        
                        phi_flt(100,1) = attitude(1,1);
                        theta_flt(100,1) = attitude(2,1);
                        
                        disp(['time: ',num2str(time(100,1)),', ' ...
                            'gx: ',num2str(gx(100,1)),', ' ...
                            'gy: ', num2str(gy(100,1)),', ' ...
                            'gz: ', num2str(gz(100,1)),', ' ...
                            'ax: ', num2str(ax(100,1)),', ' ...
                            'ay: ', num2str(ay(100,1)),', ' ...
                            'az: ', num2str(az(100,1))])
                    end

                    h1.XData = time;
                    h1.YData = phi_acc;
                    h2.XData = time;
                    h2.YData = phi_flt;
                    h3.XData = time;
                    h3.YData = theta_acc;
                    h4.XData = time;
                    h4.YData = theta_flt;
                    h5.XData = time;
                    h5.YData = phi_gyro;
                    h6.XData = time;
                    h6.YData = theta_gyro;                                                            
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
