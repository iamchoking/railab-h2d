syms x1 y1 x2 y2 theta_d
syms l

sol = solve(l*sin(theta_d)-y1 == (y2-y1)/(x2-x1)*(l*cos(theta_d)-x1),l);

subs(sol,{x1 y1 x2 y2 theta_d},{1,0,0,1,deg2rad(225)})