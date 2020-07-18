% 3DOF horizontal Robot, need to find lagrangian model
clear all
clc


% First of all we have to find all the Ti and sum them

% Ti = 1/2 mi|vi|^2 + 1/2 w^T I w
syms t m me L real
syms I g0
syms q1(t) q2(t) q3(t)

% 
% m = 10
% me = 2
% L = 1
% I = 10/12
I = m*L^2/12

q1_dot = diff(q1)
q2_dot = diff(q2)
q3_dot = diff(q3)

q = [q1(t); q2(t); q3(t)]
q_dot = [q1_dot(t); q2_dot(t); q3_dot(t)]

pc1 = [
    cos(q1)*L/2;
    sin(q1)*L/2;
    0
]
pc1 = pc1(t);
vc1 = diff(pc1)

T1 = (1/2)*m*(vc1.'*vc1) + (1/2) * q1_dot^2 * I
T1 = expand(T1)

pc2 = [cos(q1)*L + cos(q1+q2)*L/2; 
    sin(q1)*L + sin(q1+q2)*L/2;
    0]
pc2 = pc2(t);
vc2 = diff(pc2)

T2 = (1/2) * m * (vc2.' * vc2) + (1/2) * (q1_dot + q2_dot)^2 * I
T2 = expand(T2);

pc3 = [
    cos(q1)*L + cos(q1+q2)*L + cos(q1+q2+q3)*L/2; 
    sin(q1)*L + sin(q1+q2)*L + sin(q1+q2+q3)*L/2;
    0
   ]
pc3 = pc3(t)
vc3 = diff(pc3)

T3 = (1/2) * m * (vc3.' * vc3) + (1/2) * (q1_dot + q2_dot + q3_dot)^2 * I
T3 = expand(T3)


% pc4 = [
%     cos(q1)*L + cos(q1+q2)*L + cos(q1+q2+q3)*L; 
%     sin(q1)*L + sin(q1+q2)*L + sin(q1+q2+q3)*L;
%     0
%    ]
% pc4 = pc4(t)
% vc4 = diff(pc4)
% 
% T4 = (1/2) * me * (vc4.' * vc4) + (1/2) * (q1_dot + q2_dot + q3_dot)^2 * I
% T4 = simplify(T4)

T = T1 + T2 + T3 %+ T4
T = simplify(T)
T = expand(T)
M = get_M(T, q_dot)

% U = -(m2*g0*pc2(1) + m3*g0*pc3(1))

% syms s1 s2 s3 c1 c2 c3 q1_dot q2_dot q3_dot

% M_corta = subs(M, [sin(q1), sin(q2), sin(q3), cos(q1) cos(q2), cos(q3), diff(q1,t), diff(q2,t), diff(q3,t)],[s1 s2 s3 c1 c2 c3 q1_dot q2_dot q3_dot])
[c,C] = get_christoffel(M, [q1(t);q2(t);q3(t)], q_dot)
% c_corta = subs(c, [sin(q1), sin(q2), sin(q3), cos(q1) cos(q2), cos(q3), diff(q1,t), diff(q2,t), diff(q3,t)],[s1 s2 s3 c1 c2 c3 q1_dot q2_dot q3_dot])
% g = functionalDerivative(U, [q1;q2;q3])
% g_corta = subs(g, [sin(q1), sin(q2), sin(q3), cos(