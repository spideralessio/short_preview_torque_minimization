function []=MBSIWPD(long, dt)
% clear all
% clc
% close all

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

q = q0;
qd = qd0;
qdd = qdd0;

KP = 10*eye(2);
KD = 1*eye(2);

rd = [0;0];
r = points(1,:).';

p = 100;
dp = p*dt; %T = pTs

w1 = 1
w2 = 1

D = 10*eye(3)

for i = 1:count
    disp(i)
    disp(count)
    
    if count - i < p
        p = (count -i + 2);
       dp = p*dt; 
    end
    
    rdd1 = rdd0;
    if (i>count/2) 
        rdd1 = -rdd1;
    end
       
    rdd2 = rdd0;
    if (i+p>count/2) 
        rdd2 = -rdd2;
    end
    
    
    
    J1 = eval(subs(J, [q1 q2 q3], q'));
    J2 = eval(subs(J, [q1 q2 q3], (q + qd*dp)'));
    
    x = eval(subs(p_ee, [q1 q2 q3], q.')); %end effector position
    xd = J1*qd; %end effector velocity
    
    Jd1 = eval(subs(Jd, [q1 q2 q3 qd1 qd2 qd3], vertcat(q,qd)'));  
    
    M1 = eval(subs(M, [q1 q2 q3], q.'));
    M2 = eval(subs(M, [q1 q2 q3], (q + qd*dp).'));
    
    S1 = eval(subs(S, [q1 q2 q3 qd1 qd2 qd3], vertcat(q,qd)'));
    S2 = eval(subs(S, [q1 q2 q3 qd1 qd2 qd3], vertcat(q + qd*dp,qd)'));
    
    h1 = Jd1*qd;
    h2 = (J2 - J1)*qd/dp;
    
    
    n = eval(subs(C, [q1 q2 q3 qd1 qd2 qd3], vertcat(q,qd)'));
    
    A = [
        J1, zeros(size(J1));
        J2 - J1, J2
    ];
    
    b = [
        rdd1 - h1 + KD*(rd-xd) + KP*(r-x);
        rdd2 - h2;
    ];

    Q = [
        w1*eye(3) + w2*dp^2*((S2+D*M2).')*(M2^(-2))*(S2+D*M2), w2*dp*((S2+D*M2).')*inv(M2);
        w2*dp*((S2+D*M2).')*inv(M2), w2*eye(3)
    ];

    R = [
        w1*inv(M1).'*((S1+D*M1)*qd) + w2*dp*((S2+D*M2).')*(M2^(-2))*(S2+D*M2)*qd;
        w2*inv(M2).'*((S2+D*M2)*qd)
    ];


    Q_inv = inv(Q);
    pAq = Q_inv*(A.')*inv(A*Q_inv*(A.')) ;
    
    qdd = pAq*b - (eye(6) - pAq*A)*Q_inv*R; %#ok<*MINV>
    
    qdd = qdd(1:3);
    ts(:, i) = M1*qdd + S1*qd;

    [q, qd] = update_robot(qdd, q, qd, dt);
    qs(:,i) = q;
    qds(:,i) = qd;
    qdds(:,i) = qdd;
    xs(:,i) = x;
    xds(:,i) = xd;
    
    r = r + rd*dt +rdd1*dt^2;
    rd = rd + rdd1*dt;
    
    
end

save mats/MBSIWPD.mat qs qds qdds ts 
end