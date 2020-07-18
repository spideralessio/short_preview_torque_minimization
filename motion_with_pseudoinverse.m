clear all
clc


syms q1 q2 q3 qd1 qd2 qd3 real


robot = rigidBodyTree('DataFormat','column','MaxNumBodies',4);

L = 0.3

body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');


body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');


body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform([L,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link2');


body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link3');

p = [
    L*cos(q1)+L*cos(q1+q2)+L*cos(q1+q2+q3);
    L*sin(q1)+L*sin(q1+q2)+L*sin(q1+q2+q3);
    0
]
J = jacobian(p, [q1 q2 q3]);
Jd = diff(J, q1)*qd1 + diff(J, q2)*qd2 + diff(J, q3)*qd3


showdetails(robot)

dt = 0.05
T = 2
t = (0:dt:T)'; % Time
count = length(t);

% generate circle
% center = [0 0 0];
% radius = 0.4;
% theta = t*(2*pi/t(end));
% points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

% generate line
x1 = 0.25
x2 = 0.5
S = x2 - x1
xs = (x1:dt:x2)'
points = zeros(length(xs), 3);
points(:,1) = xs;
points(:,2) = xs;%*0 + 0.25;


% q0 = homeConfiguration(robot);
q0 = [0;0;0];
qd0 = [0;0;0];
qdd0 = [0;0;0];

ik = inverseKinematics('RigidBodyTree', robot);
q0 = ik('tool', trvec2tform(points(1,:)), [0,0,0,1,1,0,], q0);

ndof = length(q0);
qs = zeros(ndof, count);
qds = zeros(ndof, count);
ts = zeros(ndof, count);
rdd0 = [4*S/(T^2);4*S/(T^2);0]

q = q0;
qd = qd0;
for i = 1:count
    rdd = rdd0;
    if (i>count/2) 
        rdd = -rdd;
    end
    
    J_sub = eval(subs(J, [q1 q2 q3], q'));
    Jd_sub = eval(subs(Jd, [q1 q2 q3 qd1 qd2 qd3], vertcat(q,qd)'));
    pJ = pinv(J_sub);
    qdd = pJ*(rdd - Jd_sub*qd);
    ts(:,i) = robot.inverseDynamics(q, qd, qdd);
    [q, qd] = update_robot(qdd, q, qd, dt);
    qs(:,i) = q;
    qds(:,i) = qd;
    
end

figure
show(robot,qs(:,1));
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-1 1 -1 1])

framesPerSecond = count/T;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(:,i),'PreservePlot',false);
    drawnow
    waitfor(r);
end
hold off
tiledlayout(1,3)
nexttile
plot(ts(1,:))
nexttile
plot(ts(2,:))
nexttile
plot(ts(3,:))
