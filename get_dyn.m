function dyn = get_dyn(predyn,syn,pos_input,mode)

    if nargin < 4
        % mode: pos, q (todo), th
        mode = "pos";
    end
    q1_max  = pi*(syn.r1m + syn.r1p + syn.r1d); %set crude max/min values (for numerical solver)
    q2_max  = pi*(syn.r2m);
    
    syms q1 q2
    q = [q1;q2];
    syms th1 th2 th3
    th = [th1;th2;th3];


    % method 01: invkin wrt q
    % % be advised, vpasolve ignores all assumptions...
    % invkin_eqns = simplify(subs([predyn.forw_q_pos(1:2) == pos_input],syn));

    % disp(invkin_eqns)
    % % disp(syn)
    % if(sum(pos_input == [0;syn.l1+syn.l2+syn.l3]) == 2)
    %     % special case (full extension)
    %     q_syn = [0;0];
    %     singular = true;
    %     disp("[DYN] **This position is full-extension")
    % else
    %     q_sol   = vpasolve(invkin_eqns,[q1,q2],[0,q1_max;-q2_max*0.5,q2_max]);
    %     q_syn   = [q_sol.q1;q_sol.q2];
    % end
    % q sets the rest of the variables
    % a_syn   = simplify(subs(subs(predyn.inv0_q_a  ,syn),q,q_syn)); %motor position in rad
    % th_syn  = simplify(subs(subs(predyn.forw1_q_th,syn),q,q_syn));
    % pos_syn = simplify(subs(subs(predyn.forw_q_pos,syn),q,q_syn)); %verify loop closure & get theta

    % method 02: invkin wrt theta
    if mode == "pos"
        invkin_eqns = simplify(subs([predyn.forw2_th_pos(1) == pos_input(1),predyn.forw2_th_pos(2) == pos_input(2),syn.rxp*th2 == syn.rxd*th3],syn));
        th_sol   = vpasolve(invkin_eqns,[th1,th2,th3],[0,deg2rad(90);0,deg2rad(110);0,deg2rad(90)]);
        th_syn = [th_sol.th1;th_sol.th2;th_sol.th3];
    elseif mode == "th"
        th_syn = pos_input;
    end
    q_syn  = simplify(subs(subs(predyn.inv1b_th_q,syn),th,th_syn));
    a_syn   = simplify(subs(subs(predyn.inv0_q_a  ,syn),q,q_syn)); %motor position in rad
    pos_syn = simplify(subs(subs(predyn.forw_q_pos,syn),q,q_syn)); %verify loop closure & get theta

    % disp(pos_syn)
    
    [syn.a1,syn.a2] = deal(a_syn(1),a_syn(2));
    [syn.q1,syn.q2] = deal(q_syn(1),q_syn(2));
    [syn.th1,syn.th2,syn.th3] = deal(th_syn(1),th_syn(2),th_syn(3));
    [syn.x,syn.y,syn.theta] = deal(pos_syn(1),pos_syn(2),pos_syn(3));
    
    % disp(simplify(subs(pos,var_symbols,var_values)))
    
    % substitute constraints / obtain full dynamics
    dyn.J_fing         = double(simplify(subs(predyn.J_fing,syn)));
    % disp(dyn.J_fing);
    if(det(dyn.J_fing) == 0)
        disp("[DYN] ! Singular Solution. Skipping forward dynamics calculation")
        singular = true;
    else
        singular = false;
    end

    dyn.q              = double(q_syn);
    dyn.th             = double(th_syn);
    dyn.a              = double(a_syn);
    dyn.pos            = double(pos_syn);
    dyn.J_full         = double(simplify(subs(predyn.J_full,syn)));
    dyn.id12_fxfy_tauq = simplify(subs(predyn.id12_fxfy_tauq,syn));
    dyn.id_fxfy_taua   = simplify(subs(predyn.id_fxfy_taua,syn));
    dyn.forw_joint_pos = simplify(subs(predyn.forw_joint_pos,syn));
    dyn.f_ant          = simplify(subs(predyn.f_ant,syn));
    dyn.tauq_ant       = simplify(subs(predyn.tauq_ant,syn));
    dyn.id_fxfy_taux   = simplify(subs(predyn.id_fxfy_taux,syn));

    dyn.singular       = singular;
    if(singular) %singular solutions have diverging forward dynamics
        dyn.fd_taua_fxfy   = 0;
    else
        dyn.fd_taua_fxfy   = simplify(subs(predyn.fd_taua_fxfy,syn));
    end

end

