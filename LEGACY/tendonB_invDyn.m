% Constant Dimensions

l_prox = 0.05;
l_mid  = 0.035;
l_dist = 0.025;
l_1 = l_prox;
l_2 = l_mid;
l_3 = l_dist;

% Adjustable Dimensions (pulley dims)

r_1m = 5e-3;
r_1p = 9e-3;
r_1d = 6e-3;
r_0m = 13.5e-3;
r_0p = 11e-3;
r_2m = 20e-3;
r_xp = 5.85e-3;
r_xd = 7.15e-3;

% Spring Constants
k_0  = 300; % typical for 5mm dia / 20mm length
k_x  = 0.0; % not used (yet)
T_I0 = 0.5; % available for 5mm dia / 20mm length
T_Ix = 0.0; % not used (yet)

% Ball Screw Dimensions
e_R = 0.90  ; % conservative estimate
l   = 1e-3; % conservative pitch

% Generalized Coord. (string displacements)

q1 = 0e-3;
q2 = 0e-3;
% q2 = (pi/2)*r_2m;
% q1 = q2/r_2m*r_1m;
% (pi/2)*r_2m % = 31mm

% Output Force (x y)

F_out = [20;0];

% Assumed Displacements(s)

qx = 0e-3;

% Linear joint angle maps
D_m1 = 0;
D_m2 = 1/r_2m;
D_p1 = 1/(r_1p+r_1d*(r_xp/r_xd));
D_p2 = - r_1m/r_2m * D_p1;
D_d1 = 1/(r_1p*(r_xd/r_xp)+r_1d);
D_d2 = - r_1m/r_2m * D_d1;

D = [D_m1 D_m2;D_p1 D_p2; D_d1 D_d2]

th = D*[q1;q2];
th_deg = th*180/pi

% Kinematics

s1   = sin(th(1));
s12  = sin(th(1)+th(2));
s123 = sin(th(1)+th(2)+th(3));
c1   = cos(th(1));
c12  = cos(th(1)+th(2));
c123 = cos(th(1)+th(2)+th(3));

x = l_1*s1+l_2*s12+l_3*s123;
y = l_1*c1+l_2*c12+l_3*c123;
theta = pi/2-(th(1)+th(2)+th(3));

pos = [x;y;theta]

% Calculated Displacements
q_0 = r_0m*th(1) + r_0p*th(2);

% Jacobian (2x2)

J_thv = [
    l_1*c1+l_2*c12+l_3*c123 l_2*c12+l_3*c123 l_3*c123;
    -(l_1*s1+l_2*s12+l_3*s123) -(l_2*s12+l_3*s123) -(l_3*s123);
    -1 -1 -1
    ];

J_thv_x = J_thv(1:2,:)

J_fing = J_thv_x*D

% Antagonistic Forces

T_0 = T_I0+k_0*q_0;
tau_ant = transpose(D)*[T_0*r_0m;T_0*r_0p;0]
tau_q = transpose(J_fing)*F_out + tau_ant

% Motor Dynamics (Ball Screw)

tau_motor = l/(2*pi*e_R) * tau_q
