clear 
clc
close all
plots = {'MTN', 'MBP', 'MTIWN', 'MBIWP', 'MTSIWN', 'MBSIWP'}

gcf = figure

subplot(1,2,1)
hold on
dt = 0.001;
T = 1.8221;
t = 0:dt:T;
for i=1:length(plots)
    load(join([plots{i}, '/data.mat']))
    plot(t, sqrt(sum(qds.^2)))
end

xlabel('time')
ylabel('joint velocity norm')
% legend(plots, 'Location', 'northwest')

subplot(1,2,2)
hold on
for i=1:length(plots)
    load(join([plots{i}, '/data.mat']))
    plot(t, sqrt(sum(ts.^2)))
end
xlabel('time')
ylabel('torque norm')
le = legend(plots, 'Orientation', 'horizontal')
set(le, 'Position', [0.4 0 0.2 0.05]);
set(gcf,'position',[100 100 1800 800])
print(gcf, 'imgs/vel_torque_plots.png','-dpng', '-r150')
gcf = figure
legend(plots, 'Location', 'northwest')
for j=1:3 
    %disp(j)
    x = (j-1)*3 + 1
    subplot(3, 3, x)
    
    hold on
    dt = 0.001;
    T = 1.8221;
    t = 0:dt:T;
    for i=1:length(plots)
        load(join([plots{i}, '/data.mat']))
        plot(t, qs(j,:))
    end

    xlabel('time')
    ylabel(strcat('joint',  int2str(j), ' value'))
    

    subplot(3, 3, x+1)
    hold on
    dt = 0.001;
    T = 1.8221;
    t = 0:dt:T;
    for i=1:length(plots)
        load(join([plots{i}, '/data.mat']))
        plot(t, qds(j,:))
    end

    xlabel('time')
    ylabel(strcat('joint', int2str(j), ' velocity'))
    %legend(plots, 'Location', 'northwest')

    subplot(3, 3, x+2)
    hold on
    for i=1:length(plots)
        load(join([plots{i}, '/data.mat']))
        plot(t, ts(j,:))
    end
    xlabel('time')
    ylabel(strcat('joint', int2str(j), ' torque'))
    %legend(plots, 'Location', 'northwest')
end
le = legend(plots, 'Orientation', 'horizontal')
set(le, 'Position', [0.4 0 0.2 0.05]);
set(gcf,'position',[100 100 1800 800])
print(gcf, 'imgs/joints_plots.png','-dpng', '-r150')
