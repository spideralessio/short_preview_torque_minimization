clear all
clc
close all

syms q1 q2 q3 qd1 qd2 qd3 real


robot = rigidBodyTree('DataFormat','column','MaxNumBodies',4);

L = 1

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

M = [
30*cos(q2) + 10*cos(q3) + 10*cos(q2 + q3) + 40,     15*cos(q2) + 10*cos(q3) + 5*cos(q2 + q3) + 50/3,    5*cos(q3) + 5*cos(q2 + q3) + 10/3;
15*cos(q2) + 10*cos(q3) + 5*cos(q2 + q3) + 50/3,	10*cos(q3) + 50/3,                                  5*cos(q3) + 10/3;
5*cos(q3) + 5*cos(q2 + q3) + 10/3,                  5*cos(q3) + 10/3,                                   10/3
]

C =  [
- 5*sin(q2 + q3)*qd2^2 - 5*sin(q2 + q3)*qd3^2 - 15*sin(q2)*qd2^2 - 5*sin(q3)*qd3^2 - 30*sin(q2)*qd1*qd2 - 10*sin(q3)*qd1*qd3 - 10*sin(q3)*qd2*qd3 - 10*sin(q2 + q3)*qd1*qd2 - 10*sin(q2 + q3)*qd1*qd3 - 10*sin(q2 + q3)*qd2*qd3;
5*sin(q2 + q3)*qd1^2 + 15*sin(q2)*qd1^2 - 5*sin(q3)*qd3^2 - 10*sin(q3)*qd1*qd3 - 10*sin(q3)*qd2*qd3;
5*sin(q2 + q3)*qd1^2 + 5*sin(q3)*qd1^2 + 5*sin(q3)*qd2^2 + 10*sin(q3)*qd1*qd2;
]

S = [ 
- (5*sin(q3) + 5*sin(q2 + q3))*qd3 - (15*sin(q2) + 5*sin(q2 + q3))*qd2,     - (5*sin(q3) + 5*sin(q2 + q3))*qd3 - (15*sin(q2) + 5*sin(q2 + q3))*qd1 - (15*sin(q2) + 5*sin(q2 + q3))*qd2, -5*(sin(q3) + sin(q2 + q3))*(qd1 + qd2 + qd3);
(15*sin(q2) + 5*sin(q2 + q3))*qd1 - 5*sin(q3)*qd3,                          - 5*sin(q3)*qd3,                                                                                            -5*sin(q3)*(qd1 + qd2 + qd3);
(5*sin(q3) + 5*sin(q2 + q3))*qd1 + 5*sin(q3)*qd2,                             5*sin(q3)*(qd1 + qd2),                                                                                     0;
]

p_ee = [
    L*cos(q1)+L*cos(q1+q2)+L*cos(q1+q2+q3);
    L*sin(q1)+L*sin(q1+q2)+L*sin(q1+q2+q3);
]

J = jacobian(p_ee, [q1; q2; q3]);
Jd = diff(J, q1)*qd1 + diff(J, q2)*qd2 + diff(J, q3)*qd3


showdetails(robot)

dt = 0.001

T = 1.8221
t = (0:dt:T)'; % Time
count = length(t);


% generate line
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


qd0 = [0;0;0];
qdd0 = [0;0;0];


q0 = [
    -45*pi/180;
    135*pi/180;
    -135*pi/180
]


ndof = length(q0);
xs = zeros(2, count);
xds = zeros(2, count);
qs = zeros(ndof, count);
qds = zeros(ndof, count);
qdds = zeros(ndof, count);
ts = zeros(ndof, count);
rdd0 = [4*Sx/(T^2);4*Sy/(T^2)]

q = q0;
qd = qd0;
qdd = qdd0;

KP = 10*eye(2);
KD = 1*eye(2);

rd = [0;0];
r = points(1,:).';

D = 10*eye(3);

for i = 1:count
    disp(i)
    disp(count)
    rdd = rdd0;
    if (i>count/2) 
        rdd = -rdd;
    end
       

    J_sub = eval(subs(J, [q1 q2 q3], q'));
    x = eval(subs(p_ee, [q1 q2 q3], q.')); %end effector position
    xd = J_sub*qd; %end effector velocity
    
    Jd_sub = eval(subs(Jd, [q1 q2 q3 qd1 qd2 qd3], vertcat(q,qd)'));  
    
    M_sub = eval(subs(M, [q1 q2 q3], q.'));
    n = eval(subs(C, [q1 q2 q3 qd1 qd2 qd3], vertcat(q,qd)'));
    
    A = J_sub;
    
    R = M_sub*n + M_sub*D*M_sub*qd;

    
    b = (rdd - Jd_sub*qd) + KD*(rd-xd) + KP*(r-x);
    Q = M_sub^2;
    Q_inv = inv(Q);
    pAq = Q_inv*(A.')*inv(A*Q_inv*(A.')) ;

    
    qdd = pAq*b - (eye(3) - pAq*A)*Q_inv*R; %#ok<*MINV>

    ts(:, i) = M_sub*qdd + n;
    [q, qd] = update_robot(qdd, q, qd, dt);
    qs(:,i) = q;
    qds(:,i) = qd;
    qdds(:,i) = qdd;
    xs(:,i) = x;
    xds(:,i) = xd;
    
    r = r + rd*dt +rdd*dt^2;
    rd = rd + rdd*dt;
    
    
end

figure
show(robot,qs(:,1));
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([0 2.5 -1.5 1])

% framesPerSecond = count/T;
% rate = rateControl(framesPerSecond);
% for i = 1:count
%     show(robot,qs(:,i),'PreservePlot',false);
%     drawnow
%     waitfor(rate);
% end
tmp = 0;
for i = 1:count
    if tmp == 0
        show(robot,qs(:,i));
    end
    tmp = tmp+dt;
    if tmp > T/20
        tmp = 0;
    end
end

figure
tiledlayout(1,2)
nexttile
plot(sqrt(sum(qds.^2)))
nexttile
plot(sqrt(sum(ts.^2)))