function []=gen_plot(long, dt)
% clear all
% clc
% close all
plots = {'MTN', 'MBP', 'MTIWN', 'MBIWP', 'MTSIWN', 'MBSIWP', 'MTND', 'MBPD', 'MTIWND', 'MBIWPD', 'MTSIWND', 'MBSIWPD'}
plots = {'RT'}
long=1
dt=0.1

gcf = figure

subplot(1,2,1)
hold on


A = 1;
rdd0 = [A;A]

if long == 1
    l = 0.83
else
    l = 0.2
end
Sx = l;
Sy = l;

T = sqrt(4*l/A);
t = (0:dt:T)'; % Time
count = length(t);
% generate line
x1 = 1.4142
x2 = x1 + Sx
dx = Sx/count;
xs = (x1:dx:x2)'
y1 = -0.4142
y2 = y1 + Sy
dy = Sy/count;
ys = (y1:dy:y2)'

for i=1:length(plots)
    load(join(['mats/', plots{i}, '.mat']))
    plot(t, sqrt(sum(qds.^2)))
end

xlabel('time')
ylabel('joint velocity norm')
% legend(plots, 'Location', 'northwest')

subplot(1,2,2)
hold on
for i=1:length(plots)
    load(join(['mats/', plots{i}, '.mat']))
    plot(t, sqrt(sum(ts.^2)))
end
xlabel('time')
ylabel('torque norm')
le = legend(plots, 'Orientation', 'horizontal')
set(le, 'Position', [0.4 0 0.2 0.05]);
set(gcf,'position',[100 100 1800 800])
set(findall(gcf, 'Type', 'Line'), 'LineWidth', 2)
print(gcf, 'imgs/vel_torque_plots.png','-dpng', '-r150')

gcf = figure
work = zeros(1, length(plots))
barplots = {'MTN', 'MTIWN', 'MTSIWN', 'MBP', 'MBIWP', 'MBSIWP', 'MTND', 'MTIWND', 'MTSIWND', 'MBPD', 'MBIWPD', 'MBSIWPD'}

for i=1:length(barplots)
    load(join(['mats/', barplots{i}, '.mat']))
    dqs = [qs(:, length(qs)), qs(:,1:length(qs)-1)];
    dqs = qs - dqs;
    disp(plots{i})
    disp(sum(sum(abs(ts.*dqs))))
    work(i) = sum(sum(abs(ts.*dqs)));
end
work = reshape(work, 3, 4)
bar(work)
le = legend({'Min Norm', 'Min Short Prev', 'Min Damped Norm', 'Min Damped Short Prev'}, 'Location', 'northeast')
ylabel('Total Work [N rad]')
% set(le, 'Position', [0.4 0 0.2 0.05]);
set(gca, 'xticklabel', {'||\tau||^2', '||\tau||^2_{M^{-1}}', '||\tau||^2_{M^{-2}}'})
% xtickangle(gca, 45)
% set(findall(gcf, 'Type', 'Line'), 'LineWidth', 2)
print(gcf, 'imgs/work.png','-dpng', '-r150')


gcf = figure
imp = zeros(1, length(plots))
barplots = {'MTN', 'MTIWN', 'MTSIWN', 'MBP', 'MBIWP', 'MBSIWP', 'MTND', 'MTIWND', 'MTSIWND', 'MBPD', 'MBIWPD', 'MBSIWPD'}

for i=1:length(barplots)
    load(join(['mats/', barplots{i}, '.mat']))
    disp(plots{i})
    disp(sum(sum(abs(ts.*dt))))
    imp(i) = sum(sum(abs(ts.*dt)));
end
imp = reshape(imp, 3, 4)
bar(imp)
le = legend({'Min Norm', 'Min Short Prev', 'Min Damped Norm', 'Min Damped Short Prev'}, 'Location', 'northwest')
ylabel('Total Impulse [N s]')
% set(le, 'Position', [0.4 0 0.2 0.05]);
set(gca, 'xticklabel', {'||\tau||^2', '||\tau||^2_{M^{-1}}', '||\tau||^2_{M^{-2}}'})
% xtickangle(gca, 45)
% set(findall(gcf, 'Type', 'Line'), 'LineWidth', 2)
print(gcf, 'imgs/impulse.png','-dpng', '-r150')

gcf = figure
legend(plots, 'Location', 'northwest')
for j=1:3 
    %disp(j)
    x = (j-1)*3 + 1
    subplot(3, 3, x)
    
    hold on

    for i=1:length(plots)
        load(join(['mats/', plots{i}, '.mat']))
        plot(t, qs(j,:))
    end

    xlabel('time')
    ylabel(strcat('joint',  int2str(j), ' value'))
    

    subplot(3, 3, x+1)
    hold on

    for i=1:length(plots)
        load(join(['mats/', plots{i}, '.mat']))
        plot(t, qds(j,:))
    end

    xlabel('time')
    ylabel(strcat('joint', int2str(j), ' velocity'))
    %legend(plots, 'Location', 'northwest')

    subplot(3, 3, x+2)
    hold on
    for i=1:length(plots)
        load(join(['mats/', plots{i}, '.mat']))
        plot(t, ts(j,:))
    end
    xlabel('time')
    ylabel(strcat('joint', int2str(j), ' torque'))
    %legend(plots, 'Location', 'northwest')
end
le = legend(plots, 'Orientation', 'horizontal')
set(le, 'Position', [0.4 0 0.2 0.05]);
set(gcf,'position',[100 100 1200 800])
set(findall(gcf, 'Type', 'Line'), 'LineWidth', 2)
print(gcf, 'imgs/joints_plots.png','-dpng', '-r150')
end