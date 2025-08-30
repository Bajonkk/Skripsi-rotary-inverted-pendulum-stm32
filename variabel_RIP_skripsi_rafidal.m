%% THIS IS CODE FOR INVERTED POSITION, 
%% TO CHANGE IT INTO A NORMAL GRAVITATIONAL PENDULUM, Add negative sign to A32 and A42 
clear
clc
close all

m0 = 0.1675;
m1 = 0.0225;
m2 = 0.0235;
L1 = 0.10;
L2 = 0.225;
l1 = 0.05;
l2 = 0.1125;
J1 = 0.00041289;
J2 = 0.00037682;
C1 = 0.008;
C2 = 0.0005;
g = 9.79;
Kt = 1.54;
Kg = 0.0373;
Ku = 0.0001846;
Rm = 10.91;

x0 = [-0.05*pi;0.1*pi;0;pi];

den = J2*m2*L1^2 + J1*m2*l2^2 + J1*J2 + 2*(m2*L1*l2)^2;
a32 = (m2^2)*g*(l2^2)*L1/den;
a33 = -(C1 + (Kt*Kg/Rm))*(J2+m2*(l2^2))/den;
a34 = -(C2*m2*L1*l2)/den;
u31 = (J2+m2*(l2^2))*(Kt*Ku/Rm)/den;
a42 = (m2*g*l2*(J1+m2*(L1^2)))/den;
a43 = (m2*L1*l2)*(C1+(Kt*Kg/Rm))/den;
a44 = -C2*(J1+m2*(L1^2))/den;
u41 = -(m2*L1*l2*Kt*Ku/Rm)/den; 

A = [0 0 1 0;
    0 0 0 1;
    0 a32 a33 a34;
    0 a42 a43 a44]

B = [0;
    0;
    u31;
    u41]

C = [1 0 0 0; 0 1 0 0]

D = 0

sys = ss(A,B,C,D);
Pc = rank(ctrb(sys))
Po = rank(obsv(sys))

Q = [135000 0 0 0;
    0 10000000 0 0;
    0 0 100 0;
    0 0 0 1000];
R = 0.001;

K = lqr(A,B,Q,R);

%this in discrete
Ts = 0.000064;
sys_d = c2d(sys,Ts);

Pc_d = rank(ctrb(sys_d));
Po_d = rank(obsv(sys_d));

Ad = sys_d.a;
Bd = sys_d.b;
Cd = sys_d.c;
Dd = sys_d.d;

Kd = dlqr(Ad,Bd,Q,R)