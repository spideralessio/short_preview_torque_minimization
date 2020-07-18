clear 
clc
close all
plots = {'MTN', 'MBP', 'MTIWN', 'MBIWP', 'MTSIWN', 'MBSIWP'}

figure
tiledlayout(1,2)

nexttile
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
legend(plots, 'Location', 'northwest')

nexttile
hold on
for i=1:length(plots)
    load(join([plots{i}, '/data.mat']))
    plot(t, sqrt(sum(ts.^2)))
end
xlabel('time')
ylabel('torque norm')
legend(plots, 'Location', 'northwest')


