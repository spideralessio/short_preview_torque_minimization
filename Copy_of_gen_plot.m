clear all
clc
close all
plots = {'MTN', 'MBP', 'MTIWN', 'MBIWP', 'MTSIWN', 'MBSIWP', 'MTND', 'MBPD', 'MTIWND', 'MBIWPD', 'MTSIWND', 'MBSIWPD'}




long = 1
dt = 0.001
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

work = zeros(1, length(plots))
for i=1:length(plots)
    load(join(['mats/mats_long/', plots{i}, '.mat']))
    dqs = [qs(:, length(qs)), qs(:,1:length(qs)-1)];
    dqs = qs - dqs;
    disp(plots{i})
    disp(sum(sum(abs(ts.*dqs))))
    work(i) = sum(sum(abs(ts.*dqs)));
end
figure('Menu', 'none', 'ToolBar', 'none')
bar(work)
set(gca, 'xticklabel', plots)
xtickangle(gca, 45)


pl