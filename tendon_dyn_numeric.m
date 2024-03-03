function dyn = tendon_dyn_numeric(syn,pos_input,mode)
    %% Initialize
    % clear

    if(abs(syn.concept) == 1)
        C_LETTER = "B";
    else
        C_LETTER = "C";
    end
    % disp("[TENDON-NUMERIC] Solving NUMERIC Dynamics for Concept ["+C_LETTER+"]");

    %% Kinematic Variables / Equations
    % disp("<"+C_LETTER+">[EQN ] Setting Governing Equations...")
    
    syms a1_sym a2_sym % a (alpha): motor positions (angle)
    
    syms q1_sym q2_sym delx_sym % q: actuated deflections / del: passive (spring-load) deflections
    
    syms th1_sym th2_sym th3_sym
    
    syms x_sym y_sym theta_sym
    
    % Additional Symbols (for convenience)
    a_sym = [a1_sym;a2_sym];
    q_sym = [q1_sym;q2_sym];
    th_sym = [th1_sym;th2_sym;th3_sym];
    pos_sym = [x_sym,y_sym,theta_sym];

    s1_sym   = sin(th_sym(1));
    s12_sym  = sin(th_sym(1)+th_sym(2));
    s123_sym = sin(th_sym(1)+th_sym(2)+th_sym(3));
    c1_sym   = cos(th_sym(1));
    c12_sym  = cos(th_sym(1)+th_sym(2));
    c123_sym = cos(th_sym(1)+th_sym(2)+th_sym(3));
    
    % string equations (step 1)
    if(abs(syn.concept) == 1)
        q1_formula     = (syn.r1m*th1_sym)+syn.r1p*th2_sym+syn.r1d*th3_sym;
        q2_formula     = syn.r2m*th1_sym;
    elseif (abs(syn.concept) == 2)
        q1_formula     = -(syn.r1m*th1_sym)+syn.r1p*th2_sym+syn.r1d*th3_sym;
        q2_formula     = syn.r2m*th1_sym;
    end

    string_eqns = [q1_sym ==  q1_formula,q2_sym == q2_formula,delx_sym == -syn.rxp*th2_sym + syn.rxd*th3_sym]; % $Concept B

    % 3link equations (step 2)
    x_formula = syn.l1*s1_sym+syn.l2*s12_sym+syn.l3*s123_sym;
    y_formula = syn.l1*c1_sym+syn.l2*c12_sym+syn.l3*c123_sym;
    theta_formula = pi/2-(th_sym(1)+th_sym(2)+th_sym(3));
    
    % disp("<"+C_LETTER+">[EQN ] Done.")
    
    %% forward Kinematics Step 0 (a -> q)
    
    % q = fk0_A * a
    fk0_A = [syn.lead1/(2*pi) 0;0 syn.lead2/(2*pi)];
    
    %% forward Kinematics Step 1 (q (&del) -> th)
    % disp("<"+C_LETTER+">[fk] Solving forward Kinematics...")
    
    fk1_sol = solve(string_eqns,[th1_sym th2_sym th3_sym]);
    fk1_q_th = [fk1_sol.th1_sym;fk1_sol.th2_sym;fk1_sol.th3_sym];

    % th = fk1_A*q + fk1_b*delx;
    % fk1_A = [subs(fk1_q_th,{q1,q2,delx},{1,0,0}) subs(fk1_q_th,{q1,q2,delx},{0,1,0})];
    fk1_A = [diff(fk1_q_th,q_sym(1)) diff(fk1_q_th,q_sym(2))];
    % fk1_b = [subs(fk1_q_th,{q1,q2,delx},{0,0,1})];
    fk1_b = [diff(fk1_q_th,delx_sym)];
    if (min(isAlways(fk1_A*[q1_sym;q2_sym]+fk1_b*delx_sym==fk1_q_th)) == 0)
        disp("<"+C_LETTER+">[fk][WARN] fk1_A and fk1_b is not closed!")
    end
    fk1_A = double(fk1_A);
    fk1_b = double(fk1_b);
            
    %% forward Kinematics Step 2 (th -> ee position)

    % also not much needed ...
    % start with th1,th2,th3 -> get x y theta
    % fk2_sol = solve(tlink_eqns,[x y theta]);
    % 
    % fk2_th_pos = [fk2_sol.x;fk2_sol.y;fk2_sol.theta];
    
    
    %% Full forward Kinematics (q (, del) / a -> position)
    % fk_q_pos = subs(fk2_th_pos,th,fk1_q_th);
    % fk_a_pos = subs(fk_q_pos,q,fk0_a_q);
    
    % outputs
    % dyn.fk2_th_pos   = simplify(subs(fk2_th_pos,syn));
    % dyn.fk_q_pos     = simplify(subs(fk_q_pos,syn));
    % dyn.fk_a_pos     = simplify(subs(fk_a_pos,syn));
    % dyn.fk_joint_pos = simplify(subs([[syn.l1*s1_sym;syn.l1*c1_sym] [syn.l1*s1_sym+syn.l2*s12_sym;syn.l1*c1_sym+syn.l2*c12_sym] [syn.l1*s1_sym+syn.l2*s12_sym+syn.l3*s123_sym;syn.l1*c1_sym+syn.l2*c12_sym+syn.l3*c123_sym]],syn));
    % each column vector is DIP, PIP, fingertip position, respectively

    % disp("<"+C_LETTER+">[fk] Done.")
    
    %% Inverse Kinematics 0 (q -> a) (TODO: NUMERIC solution is unstable, requires numercial solution)
    
    % disp("<"+C_LETTER+">[INV ] Solving Inverse Kinematics...")
    
    % a = ik0_A*q;
    ik0_A = [(2*pi)/syn.lead1 0;0 (2*pi)/syn.lead2];
    
    % disp("<"+C_LETTER+">[INV ] Done.*")

    % Inverse kinematics to recover configuration data (a,q,th,pos)
    if mode == "pos"
        ik_eqns     = [x_formula == pos_input(1),y_formula == pos_input(2),syn.rxp*th2_sym == syn.rxd*th3_sym];
        % disp(ik_eqns);
        th_sol      = vpasolve(ik_eqns,[th1_sym,th2_sym,th3_sym],[0,deg2rad(90);0,deg2rad(110);0,deg2rad(90)]);
        th      = double([th_sol.th1_sym;th_sol.th2_sym;th_sol.th3_sym]);
    elseif mode == "th"
        th = pos_input;
    end
    q   = double(subs([q1_formula;q2_formula],th_sym,th));
    a   = ik0_A*q; %motor position in rad
    pos = double(subs([x_formula;y_formula;theta_formula],th_sym,th)); %verify loop closure & get theta
    
    % disp("IK solved for pos: "+strjoin(string(pos)))
    % disp("  a  : "+strjoin(string(a)))
    % disp("  q  : "+strjoin(string(q)))
    % disp("  th : "+strjoin(string(rad2deg(th))))
    % disp("  pos: "+strjoin(string(pos)))

    % convenience variables
    th1  = th(1);
    th2  = th(2);
    th3  = th(3);
    s1   = sin(th(1));
    s12  = sin(th(1)+th(2));
    s123 = sin(th(1)+th(2)+th(3));
    c1   = cos(th(1));
    c12  = cos(th(1)+th(2));
    c123 = cos(th(1)+th(2)+th(3));

    %% Differential Kinematics 0 (adot -> qdot) (indep to position)
    
    % disp("<"+C_LETTER+">[DIFF] Solving Differential Kinematics...")

    % fk0_A = [syn.lead1/(2*pi) 0;0 syn.lead2/(2*pi)];
    J_a_q = fk0_A;
    % (1/(2*pi))*[syn.lead1 0;0 syn.lead2]; %(qdot = J_a_q * adot)
    
    % transmission force effecnency (forward)
    trans_eff_f = [syn.eff1 0;0 syn.eff2];
    
    %% Differential Kinematics 1 (qdot -> thdot) (indep to position)
    
    J_q_th = fk1_A; %(thdot = J_q_th * qdot)
    
    %% Differential Kinematics 2 (thdot -> v(xdot,ydot,thdot) ) (th needs to be supplied)
    
    J_th_v = [
          syn.l1*c1+syn.l2*c12+syn.l3*c123      syn.l2*c12+syn.l3*c123    syn.l3*c123;
        -(syn.l1*s1+syn.l2*s12+syn.l3*s123)   -(syn.l2*s12+syn.l3*s123) -(syn.l3*s123);
        -1                        -1                -1
        ];
    
    %% Differential Kinematics Combinations (adot/qdot -> v (xdot,ydot))
    
    J_fing = J_th_v(1:2,:) * J_q_th;         % v = J_fing*qdot (2x2) (qdot -> v (xdot,ydot)) -> very important!!
    J_q_v  = J_th_v * J_q_th;                % v = J_q_v*qdot  (3x2) (qdot -> v (xdot,ydot,thetadot))
    J_full = J_fing * J_a_q;                 % v = J_full*adot (2x2) (adot -> v (xdot,ydot))
    J_a_v  = J_q_v  * J_a_q;                 % v = J_full*adot (3x2) (adot -> v (xdoy,ydot,thetadot))

    if(det(J_fing) == 0)
        singular = true;
        disp("[WARN] This configuration is singular. Using pseudoinverses.")
    else
        singular = false;
    end

    % disp("<"+C_LETTER+">[DIFF] Done.")
    
        
    %% Antagonistic Forces (th needs to be supplied) *!!Different for each concept
    
    % disp("<"+C_LETTER+">[FD  ] Solving forward Dynamics...")

    if (abs(syn.concept) == 1)
        % $[syn.concept B] Tensioner passes through MCP and PIP
        
        dely      = th1*syn.rym + th2*syn.ryp;
        Ty        = syn.Tyi + syn.ky*dely;
        tauth_ant = [-Ty*syn.rym;-Ty*syn.ryp;0];
    elseif (abs(syn.concept) == 2)
        % $[syn.concept C] One tensioner in PIP
        
        dely      = th2*syn.ryp;
        Ty        =  syn.Tyi + syn.ky*dely;
        tauth_ant = [0;-Ty*syn.ryp;0];
    end
    
    tauq_ant  = transpose(J_q_th)*tauth_ant; %(always negative) this is the "offset" effect in superposition (zero dynamics)
    % disp(tauq_ant);

    %% forward Dynamics 0 (taua -> tauq) (th2 needs to be supplied)

    % tauq = fd0_A*taua
    % fd0_A         = inv(transpose(J_a_q))*trans_eff_f;
    fd0_A   = (eye(2)/transpose(J_a_q))*trans_eff_f;
    
    %% forward Dynamics 1&2 (tauq -> fx fy) (th needs to be supplied)

    % f = fd12_A*tauq + fd12_b
    % fd12_A = inv(transpose(J_fing));
    if(singular)
        fd12_A = pinv(transpose(J_fing));
    else
        fd12_A = eye(2)/transpose(J_fing);
    end
    fd12_b = fd12_A*tauq_ant;

    % disp("J_fing: ");
    % disp(J_fing);
    % disp("J_fing^-T");
    % disp(eye(2)/transpose(J_fing));
    % disp(fd12_A);

    %% Full forward Dynamics (taua -> fx fy) (th needs to be supplied) (special treatment needed for singular)
    
    % f = fd_A*taua + fd_b
    fd_A         = fd12_A*fd0_A;
    fd_b         = fd12_b;

    % disp("<"+C_LETTER+">[FD  ] Done.")
    
    %% Inverse Dynamics step 0 (tauq -> taua)
    % disp("<"+C_LETTER+">[ID  ] Solving Inverse Dynamics...")
    
    % taua = id0_A * tauq;
    % id0_A         = transpose(J_a_q)*inv(trans_eff_f);
    id0_A         = transpose(J_a_q)/trans_eff_f;
    
    %% Inverse Dynamics step 1&2 (fx fy -> tauq)
    
    %ID is quite simple

    % tauq = id12_A*f + id12_b
    id12_A         =  transpose(J_fing);
    id12_b         = -tauq_ant;

    %% Full Inverse Dynamics (tauq -> fx fy)

    % taua = id_A*f + id_b = id0_A*tauq = id0_A*(id12_A*f+id12_b)
    id_A          = id0_A*id12_A;
    id_b          = id0_A*id12_b;
    
    % disp("<"+C_LETTER+">[ID  ] Done.")
    
    %% Additoinal Formulas
    
    % getting taux from dynamics approaches

    % for the "coupler (string x)" tension %TODO: move this

    % moment exerted to dip from external force (cross product)
    % tauf_th3 = cross([syn.l3*s123;syn.l3*c123;0],[fx;fy;0]);
    % taux = 1/syn.rxd*(syn.r1d*tauq1+tauf_th3(3));
    % id_fxfy_taux = simplify(subs(dyn_taux,tauq,id12_fxfy_tauq));
    
    % taux = idx_A*f + idx_b;
    l3_skew = syn.l3 * [0 0 c123;0 0 -s123;-c123 s123 0]; %skew symmetric matrix
    idx_A = 1/syn.rxd * (syn.r1d*[1 0]*id12_A + [0 0 1]*l3_skew*[1 0;0 1;0 0]);
    idx_b = 1/syn.rxd * (syn.r1d*[1 0]*id12_b);
    
    % disp("[TENDON-NUMERIC] NUMERIC Dynamics Solved ( CONCEPT "+C_LETTER+"), Position: "+strjoin(string(pos)));

    %%%
    % return values
    dyn.singular = singular;

    dyn.q   = q;
    dyn.th  = th;
    dyn.a   = a;
    dyn.pos = pos;

    dyn.J_fing = J_fing;
    dyn.J_full = J_full;

    dyn.id12_A = id12_A;
    dyn.id12_b = id12_b;

    dyn.id_A   = id_A;
    dyn.id_b   = id_b;

    dyn.fk_joint_pos = [[syn.l1*s1;syn.l1*c1] [syn.l1*s1+syn.l2*s12;syn.l1*c1+syn.l2*c12] [syn.l1*s1+syn.l2*s12+syn.l3*s123;syn.l1*c1+syn.l2*c12+syn.l3*c123]];

    dyn.dely           = dely;
    dyn.tauq_ant       = tauq_ant;
    dyn.idx_A  = idx_A;
    dyn.idx_b  = idx_b;

    if(dyn.singular) %singular solutions have divergent forward dynamics / indeterminate antagonistic force
        dyn.fd_A           = 0;
        dyn.fd_b           = 0;
        dyn.f_ant          = 0;
    else
        dyn.fd_A           = fd_A;
        dyn.fd_b           = fd_b;
        dyn.f_ant          = fd12_b;
    end
end
    

