clc
clear
close all

t = udp('127.0.0.1', 80, 'LocalPort', 14550);
t.InputBufferSize = 89120;

fopen(t)

time = 0;
rpm = 0;
fc = 0.1;

h1 = plot(time,rpm,'-r.');
hold on
grid on
h2 = plot(time,rpm,'b');
h3 = plot(time,rpm,'LineWidth',3,'Color',[0 0 1]);

ylim([-300 300])
set(gca, 'GridLineStyle', ':');
set(gca, 'GridAlpha', 1);
set(gca, 'YTick', -300:1:300);
N = 5;
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
            case 14
                if int16(crc(A,1,170)) == get_crc(A)
                    time(i,1) = uint32_t(A,7,10);
                    rpm_target(i,1) = float(A,11,14);
                    rpm(i,1) = float(A,15,18);
                    disp(['time: ',num2str(time(i,1)),', ' ...
                        'rpm_target: ', num2str(rpm_target(i,1)),', ' ...
                        'rpm: ', num2str(rpm(i,1))])
                    
                    if isequal(i,1)
                        lowpass_flt(i,1) = rpm(i,1);
                    else
                        lowpass_flt(i,1) = lowpass_flt(i-1,1) + fc*(rpm(i,1) - lowpass_flt(i-1,1));
                        if abs(lowpass_flt(i,1)) < 1e-3
                            lowpass_flt(i,1) = 0;
                        end
                    end
                    
                    if i<=100
                        time_plot = time;
                        rpm_target_plot = rpm_target;
                        rpm_plot = rpm;
                        lowpass_flt_plot = lowpass_flt;
                    else
                        time_plot = time(size(time,1)-100:end);
                        rpm_target_plot = rpm_target(size(rpm_target,1)-100:end);
                        rpm_plot = rpm(size(rpm,1)-100:end);
                        lowpass_flt_plot = lowpass_flt(size(lowpass_flt,1)-100:end);
                    end
                    h1.XData = time_plot;
                    h1.YData = rpm_target_plot;
                    h2.XData = time_plot;
                    h2.YData = rpm_plot;
                    h3.XData = time_plot;
                    h3.YData = lowpass_flt_plot;
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
                fclose(t)
                break
        end
    end
    freq(i,1) = 1/toc;
end

fclose(t)
