syms q1 q2 delx dely
syms th1 th2 th3

syms r1m r1p r1d r2m rxp rxd ryp
syms l1 l2 l3

syms Dm1 Dm2 Dp1 Dp2 Dd1 Dd2
syms Amx Amy Apx Apy Adx Ady

q = [q1;q2];
del = [delx;dely];

string_eqns = [q1 == -(r1m*th1)+r1p*th2+r1d*th3,q2 == r2m*th1,delx == -rxp*th2 + rxd*th3];

forw_q_th_sol = solve(string_eqns,[th1 th2 th3]);
th_sym = [forw_q_th_sol.th1;forw_q_th_sol.th2;forw_q_th_sol.th3];

inv_th_q_sol = solve(string_eqns,[q1 q2 delx]);
q_sym = [inv_th_q_sol.q1;inv_th_q_sol.q2];

D_sym = [simplify(subs(th_sym,{q1,q2,delx,dely},{1,0,0,0})) simplify(subs(th_sym,{q1,q2,delx,dely},{0,1,0,0}))];
A_sym = [simplify(subs(th_sym,{q1,q2,delx,dely},{0,0,1,0})) simplify(subs(th_sym,{q1,q2,delx,dely},{0,0,0,1}))];
% isAlways(D*q+A*del==th_sol)

syms th1_set th2_set th3_set
syms s1 s12 s123 c1 c12 c123
syms x y theta pos

th = [th1_set;th2_set;th3_set];

s1   = sin(th(1));
s12  = sin(th(1)+th(2));
s123 = sin(th(1)+th(2)+th(3));
c1   = cos(th(1));
c12  = cos(th(1)+th(2));
c123 = cos(th(1)+th(2)+th(3));

forw_th_pos = [l1*s1+l2*s12+l3*s123;l1*c1+l2*c12+l3*c123;pi/2-(th(1)+th(2)+th(3))];
forw_q_pos = subs(subs(forw_th_pos,th,th_sym),delx,0);

% (unsolvable)
% kin_eqs = [x == forw_q_pos(1),y == forw_q_pos(2)]
% inv_kin_full = solve(kin_eqs,[q1 q2])

J_thv_sym = [
    l1*c1+l2*c12+l3*c123 l2*c12+l3*c123 l3*c123;
    -(l1*s1+l2*s12+l3*s123) -(l2*s12+l3*s123) -(l3*s123);
    -1 -1 -1
    ];

J_thv_x_sym = J_thv_sym(1:2,:);

J_fing_sym = J_thv_x_sym*D_sym;

syms tau_q1 tau_q2 tau_q3
syms tau_motor_1 tau_motor_2
syms T1 T2
syms Ty

syms fx fy

ballscrew_pitch = 1e-3;
ballscrew_eff = 0.9;

kx  = 300;
Tix = 0.5;

ky  = 300;
Tiy = 0.5;

qy = ryp*th2;
tau_q_ant = transpose(D_sym)*[0;Tix + qy*ky;0];

tau_q = transpose(J_fing_sym)*[fx;fy] + tau_q_ant;

tau_motor = ballscrew_pitch/(2*pi*ballscrew_eff)*tau_q;
