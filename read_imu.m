clc
clear
close all

t = udp('127.0.0.1', 80, 'LocalPort', 14550);
t.InputBufferSize = 89120;

fopen(t)

time = 0;
ax = 0;
ay = 0;
az = 0;
gx = 0;
gy = 0;
gz = 0;

subplot(3,2,1);
h1 = plot(time,ax,'-r.');
subplot(3,2,3);
h2 = plot(time,ay,'-r.');
subplot(3,2,5);
h3 = plot(time,az,'-r.');
subplot(3,2,2);
h4 = plot(time,gx,'-b.');
subplot(3,2,4);
h5 = plot(time,gy,'-b.');
subplot(3,2,6);
h6 = plot(time,gz,'-b.');

ylim([-1000 1000])
set(gca, 'GridLineStyle', ':');
set(gca, 'GridAlpha', 1);
set(gca, 'YTick', -1000:1:1000);
N = 10;
a = get(gca,'YTickLabel');
b = cell(size(a));
b(mod(1:size(a,1),N)==1,:) = a(mod(1:size(a,1),N)==1,:);
set(gca,'YTickLabel',b);
ylim([-inf inf])
                    
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
                    ax(i,1) = int16_t(A,11,12);
                    ay(i,1) = int16_t(A,13,14);
                    az(i,1) = int16_t(A,15,16);
                    gx(i,1) = int16_t(A,17,18);
                    gy(i,1) = int16_t(A,19,20);
                    gz(i,1) = int16_t(A,21,22);
                    disp(['time: ',num2str(time(i,1)),', ' ...
                        'ax: ',num2str(ax(i,1)),', ' ...
                        'ay: ', num2str(ay(i,1)),', ' ...
                        'az: ', num2str(az(i,1)),', ' ...
                        'gx: ', num2str(gx(i,1)),', ' ...
                        'gy: ', num2str(gy(i,1)),', ' ...
                        'gz: ', num2str(gz(i,1))])
                    
                    if i<=100
                        time_plot = time;
                        ax_plot = ax;
                        ay_plot = ay;
                        az_plot = az;
                        gx_plot = gx;
                        gy_plot = gy;
                        gz_plot = gz;
                    else
                        time_plot = time(size(time,1)-100:end);
                        ax_plot = ax(size(ax,1)-100:end);
                        ay_plot = ay(size(ay,1)-100:end);
                        az_plot = az(size(az,1)-100:end);
                        gx_plot = gx(size(gx,1)-100:end);
                        gy_plot = gy(size(gy,1)-100:end);
                        gz_plot = gz(size(gz,1)-100:end);
                    end
                    h1.XData = time_plot;
                    h1.YData = ax_plot;
                    h2.XData = time_plot;
                    h2.YData = ay_plot;
                    h3.XData = time_plot;
                    h3.YData = az_plot;
                    h4.XData = time_plot;
                    h4.YData = gx_plot;
                    h5.XData = time_plot;
                    h5.YData = gy_plot;
                    h6.XData = time_plot;
                    h6.YData = gz_plot;
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
