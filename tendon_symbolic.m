function dyn = tendon_symbolic(CONCEPT,syn)
    %% Initialize
    % clear
    
    if nargin < 1
        CONCEPT = 'C'; %plus or minus doesn't matter here.
    end

    if nargin < 2
        syn = struct();
    end
    
    %% Basic Geometry (symbolic)
        
    syms r1m r1p r1d r2m rxp rxd rym ryp
    syms l1 l2 l3
    
    syms lead1 lead2
    syms eff1  eff2
    
    %% Kinematic Variables / Equations
    disp("[EQN ] Setting Governing Equations...")
    
    syms a1 a2 % a (alpha): motor positions (angle)
    
    syms q1 q2 delx % q: actuated deflections / del: passive (spring-load) deflections
    % (dely is entirely dependent to th2 (& th2 in case of B))
    
    syms a1dot a2dot
    syms q1dot q2dot
    
    syms th1 th2 th3
    
    syms x y theta
    
    % Additional Symbols (for convenience)
    a = [a1;a2];
    q = [q1;q2];
    
    adot = [a1dot;a2dot];
    qdot = [q1dot;q2dot];
    
    th = [th1;th2;th3];
    s1   = sin(th(1));
    s12  = sin(th(1)+th(2));
    s123 = sin(th(1)+th(2)+th(3));
    c1   = cos(th(1));
    c12  = cos(th(1)+th(2));
    c123 = cos(th(1)+th(2)+th(3));
    
    pos = [x,y,theta];
    
    % assume(th1 >= 0 & th1 <= deg2rad(90) );
    % assume(th2 >= 0 & th2 <= deg2rad(110));
    % assume(th3 >= 0 & th3 <= deg2rad(90) );
    
    % string equations (step 1)
    if(CONCEPT(1:1) == 'B')
        string_eqns = [q1 ==  (r1m*th1)+r1p*th2+r1d*th3,q2 == r2m*th1,delx == -rxp*th2 + rxd*th3]; % $Concept B
    elseif (CONCEPT(1:1) == 'C')
        string_eqns = [q1 == -(r1m*th1)+r1p*th2+r1d*th3,q2 == r2m*th1,delx == -rxp*th2 + rxd*th3]; % $Concept C
    end
    
    % 3link equations (step 2)
    tlink_eqns = [x == l1*s1+l2*s12+l3*s123, y == l1*c1+l2*c12+l3*c123, theta == pi/2-(th(1)+th(2)+th(3))];
    tlink_eqns_pos = tlink_eqns(1:2);
    
    disp("[EQN ] Done.")
    
    %% Forward Kinematics Step 0 (a -> q)
    
    forw0_a_q = [lead1/2*pi*a1;lead2/2*pi*a2];
    
    %% Forward Kinematics Step 1 (q (&del) -> th)
    disp("[FORW] Solving Forward Kinematics...")
    
    forw1_sol = solve(string_eqns,[th1 th2 th3]);
    forw1_q_th = [forw1_sol.th1;forw1_sol.th2;forw1_sol.th3];
    
    forw1_D = [simplify(subs(forw1_q_th,{q1,q2,delx},{1,0,0})) simplify(subs(forw1_q_th,{q1,q2,delx},{0,1,0}))];
    forw1_A = [simplify(subs(forw1_q_th,{q1,q2,delx},{0,0,1}))];
    if (min(isAlways(forw1_D*[q1;q2]+forw1_A*delx==forw1_q_th)) == 0)
        disp("[FORW][WARN] forw1_D and forw1_A is not closed!")
    end
            
    %% Forward Kinematics Step 2 (th -> ee position)
    
    % start with th1,th2,th3 -> get x y theta
    forw2_sol = solve(tlink_eqns,[x y theta]);
    
    forw2_th_pos = [forw2_sol.x;forw2_sol.y;forw2_sol.theta];
    
    % (unsolvable)
    % kin_eqs = [x == forw_q_pos(1),y == forw_q_pos(2)]
    % inv_kin_full = solve(kin_eqs,[q1 q2])
    
    %% Full Forward Kinematics (q (, del) / a -> position)
    forw_q_pos = subs(forw2_th_pos,th,forw1_q_th);
    forw_a_pos = subs(forw_q_pos,q,forw0_a_q);
    
    % outputs
    dyn.forw1_q_th = simplify(subs(forw1_q_th,syn));
    dyn.forw_q_pos = simplify(subs(forw_q_pos,syn));
    dyn.forw_a_pos = simplify(subs(forw_a_pos,syn));
    dyn.forw_joint_pos = simplify(subs([[l1*s1;l1*c1] [l1*s1+l2*s12;l1*c1+l2*c12] [l1*s1+l2*s12+l3*s123;l1*c1+l2*c12+l3*c123]],syn));
    % each column vector is DIP, PIP, fingertip position, respectively

    disp("[FORW] Done.")
    
    %% Inverse Kinematics 0 (q -> a) (TODO: symbolic solution is unstable, requires numercial solution)
    
    disp("[INV ] Solving Inverse Kinematics...")
    
    inv0_q_a1 = (2*pi)/lead1 * q1;
    inv0_q_a2 = (2*pi)/lead2 * q2;
    inv0_q_a = [inv0_q_a1;inv0_q_a2];
    
    dyn.inv0_q_a = subs(inv0_q_a,syn);

    %% Inverse Kinematics 1A (th1 th2 del-> q th3)
    
    inv1a_sol = solve(string_eqns,[q1 q2 th3]);
    inv1a_q1 = inv1a_sol.q1;
    inv1a_q2 = inv1a_sol.q2;
    inv1a_th3 = inv1a_sol.th3;
    
    
    %% Inverse Kinematics 1B (th1 th2 th3-> q delx)
    
    inv1b_sol = solve(string_eqns,[q1 q2 delx]);
    inv1b_q1 = inv1b_sol.q1;
    inv1b_q2 = inv1b_sol.q2;
    inv1b_delx = inv1b_sol.delx;
    
    %% Inverse Kinematics 2A (x y -> th1 th2 (assume delx = 0))
    
    % Cannot be done symbolically
    % subs(string_eqns(3),delx,0);
    % 
    % lb_hybrid_eqns = [subs(string_eqns(3),delx,0),tlink_eqns_pos]
    % 
    % inv2a_sol = solve(tlink_eqns,[th1 th2],'ReturnConditions',true)
    % disp(inv2a_sol.conditions)
    
    %% Inverse Kinematics 2B (x y theta -> th1 th2 th3)
    
    % Cannot be done symbolically
    % inv2b_sol = solve(tlink_eqns,[th1 th2 th3])
    
    %% Full Inverse Kinematics (TODO)
    
    disp("[INV ] Done.*")
    
    
    %% Differential Kinematics 0 (adot -> qdot) (indep to position)
    
    disp("[DIFF] Solving Differential Kinematics...")
    
    J_a_q = (1/(2*pi))*[lead1 0;0 lead2]; %(qdot = J_a_q * adot)
    
    transmission_eff_forw = [eff1 0;0 eff2];
    
    %% Differential Kinematics 1 (qdot -> thdot) (indep to position)
    
    J_q_th = forw1_D; %(thdot = J_q_th * qdot)
    
    %% Differential Kinematics 2 (thdot -> v(xdot,ydot,thdot) ) (th needs to be supplied)
    
    J_th_v = [
          l1*c1+l2*c12+l3*c123      l2*c12+l3*c123    l3*c123;
        -(l1*s1+l2*s12+l3*s123)   -(l2*s12+l3*s123) -(l3*s123);
        -1                        -1                -1
        ];
    
    
    %% Differential Kinematics Combine (adot/qdot -> v (xdot,ydot))
    
    J_fing = J_th_v(1:2,:) * J_q_th;         % (qdot -> v (xdot,ydot)) (th needs to be supplied)
    J_q_v = J_th_v * J_q_th;                 % (qdot -> v (xdot,ydot,thetadot))
    J_full = J_a_q*J_fing;                   % (adot -> v (xdot,ydot))

    dyn.J_fing = simplify(subs(J_fing,syn));
    dyn.J_q_v  = simplify(subs(J_q_v ,syn));
    dyn.J_full = simplify(subs(J_full,syn));

    disp("[DIFF] Done.")
    
    
    %% Dynamic Variables
    disp("[FD  ] Solving Forward Dynamics...")
    
    syms taua1 taua2 % motor torque
    taua = [taua1;taua2];
    
    syms tauq1 tauq2 % string tension
    tauq = [tauq1;tauq2];
    
    syms tauth1 tauth2 tauth3 % finger joint moment
    tauth = [tauth1 tauth2 tauth3];
    
    syms fx fy nz % fingertip force
    f = [fx;fy;nz];
    
    % Antagonistic Constants
    
    % dely Ty
    
    syms Tyi ky
    
    %% Antagonistic Forces (th2 needs to be supplied) *!!Different for each concept
    
    if (CONCEPT(1:1) == 'B')
        % $[CONCEPT B] Tensioner passes through MCP and PIP
        
        dely = th1*rym + th2*ryp;
        Ty = Tyi + ky*dely;
        tauth_ant = [-Ty*rym;-Ty*ryp;0];
    elseif (CONCEPT(1:1) == 'C')
        % $[CONCEPT C] One tensioner in PIP
        
        dely = th2*ryp;
        Ty   =  Tyi + ky*dely;
        tauth_ant = [0;-Ty*ryp;0];
    end
    
    tauq_ant  = transpose(J_q_th)*tauth_ant; %(always negative) this is the "offset" effect in superposition (zero dynamics)
    f_ant    = simplify(inv(transpose(J_fing))*tauq_ant); % constant force representing effect of antogistic tensioners
    
    % f_ant     = inv(transpose(J_th_v))*tauth_ant; % WRONG!
    % f_ant = simplify(f_ant(1:2))
    
    % for the "coupler (string x)" tension
    dyn_taux = 1/rxd*(r1d*tauq1+norm(cross([l3*s123;l3*c123;0],[fx;fy;0])));

    dyn.f_ant = simplify(subs(f_ant,syn));
    dyn.tauq_ant = simplify(subs(tauq_ant,syn));
    
    %% Forward Dynamics 0 (taua -> tauq) (th2 needs to be supplied)
    
    fd0_taua_tauq = simplify(inv(transpose(J_a_q))*transmission_eff_forw*taua);
    
    %% Forward Dynamics 1&2 (tauq -> fx fy) (th needs to be supplied)
    
    fd12_tauq_fxfy = simplify(inv(transpose(J_fing))*(tauq+tauq_ant));
    
    %% Full forward Dynamics (taua -> fx fy) (th needs to be supplied)
    
    fd_taua_fxfy = simplify(subs(fd12_tauq_fxfy,tauq,fd0_taua_tauq));
    
    % fd_output
    dyn.fd_taua_fxfy = simplify(subs(fd_taua_fxfy,syn));

    disp("[FD  ] Done.")
    
    %% Inverse Dynamics step 0 (tauq -> taua)
    
    id0_tauq_taua = transpose(J_a_q)*inv(transmission_eff_forw)*tauq;
    
    disp("[ID  ] Solving Inverse Dynamics...")
    %% Inverse Dynamics step 1&2 (fx fy -> tauq)
    
    %ID is quite simple
    
    id12_fxfy_tauq = transpose(J_fing)*f(1:2) - tauq_ant;
    
    dyn.id12_fxfy_tauq = simplify(subs(id12_fxfy_tauq,syn));

    %% Full inverse Dynamics (tauq -> fx fy)
    
    id_fxfy_taua = simplify(subs(id0_tauq_taua,tauq,id12_fxfy_tauq));
    
    %id_output
    dyn.id_fxfy_taua = simplify(subs(id_fxfy_taua,syn));

    disp("[ID  ] Done.")
    
    %% Additoinal Formulas
    
    % getting taux from dynamics approaches
    id_fxfy_taux = simplify(subs(dyn_taux,tauq,id12_fxfy_tauq));
    fd_taua_taux = simplify(subs(subs(dyn_taux,f(1:2),fd_taua_fxfy),tauq,fd0_taua_tauq));
    
    % getting resulting joint tauth in fd&id situations
    id2_fxfy_tauth = transpose(J_th_v)*[fx;fy;nz];
    fd2_tauq_tauth = subs(id2_fxfy_tauth,f(1:2),fd12_tauq_fxfy);
end
    

