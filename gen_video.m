function []=gen_video(long, dt, framesPerSecond)
% clear all
% clc
% close all
plots = {'MTN', 'MBP', 'MTIWN', 'MBIWP', 'MTSIWN', 'MBSIWP', 'MTND', 'MBPD', 'MTIWND', 'MBIWPD', 'MTSIWND', 'MBSIWPD'}
% plots = {'RT'}
% long = 1
% dt = 0.01
% framesPerSecond = 15
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
count = length(t);
points = zeros(length(xs), 2);
points(:,1) = xs;
points(:,2) = ys;

for i=1:length(plots)
    load(join(['mats/', plots{i}, '.mat']))
    figure
    show(robot,qs(:,1));
    view(2)
    ax = gca;
    ax.Projection = 'orthographic';
    hold on
    plot(points(:,1),points(:,2),'k')
    axis([0 2.5 -1.5 1])

    nframes = T*framesPerSecond;
    step = floor(count/nframes);
    rate = rateControl(framesPerSecond);
    video = VideoWriter(join(['imgs/', plots{i}]));
    video.FrameRate = framesPerSecond;
    open(video);
    for j = 1:step:count
        show(robot,qs(:,j),'PreservePlot',false);
        drawnow
        waitfor(rate);
        frame = getframe(gcf);
        writeVideo(video,frame);
    end
    close(video)
    
    gfc = figure
    show(robot,qs(:,1));
    l = findall(gfc, 'Type', 'Text')
    delete(l(1:3))
    l = findall(gfc, 'Type', 'Line')
    delete(l(1:3))
    view(2)
    ax = gca;
    ax.Projection = 'orthographic';
    hold on
    plot(points(:,1),points(:,2),'k')
    axis([0 2.5 -1.5 1])
    for j = 1:floor(count/20):count
        show(robot,qs(:,j),'PreservePlot',true);
        drawnow
    end
    print(gfc, join(['imgs/rob_motion_',plots{i},'.png']),'-dpng', '-r150')
end

end