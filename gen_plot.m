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


load('MTSIWN/data.mat')

L = 1
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',4);
link1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint1,trvec2tform([0 0 0]));
joint1.JointAxis = [0 0 1];
link1.Joint = joint1;
link1.Mass = 10;
link1.Inertia = [1 1 10/12 0 0 0];
link1.CenterOfMass = [L/2 0 0];
addBody(robot, link1, 'base');


link2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint2, trvec2tform([L,0,0]));
joint2.JointAxis = [0 0 1];
link2.Joint = joint2;
link2.Inertia = [1 1 10/12 0 0 0];
link2.Mass = 10;
link2.CenterOfMass = [L/2 0 0];
addBody(robot, link2, 'link1');


link3 = rigidBody('link3');
joint3 = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint3, trvec2tform([L,0,0]));
joint3.JointAxis = [0 0 1];
link3.Joint = joint3;
link3.Inertia = [1 1 10/12 0 0 0];
link3.Mass = 10;
link3.CenterOfMass = [L/2 0 0];
addBody(robot, link3, 'link2');


tool = rigidBody('tool');
jointTool = rigidBodyJoint('fix1','fixed');
setFixedTransform(jointTool, trvec2tform([L, 0, 0]));
tool.Joint = jointTool;
tool.Mass = 0;
link3.Inertia =  [0 0 0 0 0 0];
addBody(robot, tool, 'link3');

count = length(t)
x1 = 1.4142
x2 = x1 + 0.83
Sx = x2 - x1
dx = Sx/count;
xs = (x1:dx:x2)'
y1 = -0.4142
y2 = y1 + 0.83
Sy = y2 - y1
dy = Sy/count;
ys = (y1:dy:y2)'

points = zeros(length(xs), 2);
points(:,1) = xs;
points(:,2) = ys;%*0 + 0.25;

figure
show(robot,qs(:,1));
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([0 2.5 -1.5 1])

framesPerSecond = 60;
nframes = T*framesPerSecond;
step = floor(count/nframes);
rate = rateControl(framesPerSecond);
video = VideoWriter('video');
video.FrameRate = framesPerSecond;
open(video);
for i = 1:step:count
    show(robot,qs(:,i),'PreservePlot',false);
    drawnow
    waitfor(rate);
    frame = getframe(gcf);
    writeVideo(video,frame);
end
close(video)
