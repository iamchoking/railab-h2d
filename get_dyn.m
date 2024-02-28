function dyn = get_dyn(predyn,syn,pos_input)

    q1_max  = pi*(syn.r1m + syn.r1p + syn.r1d); %set crude max/min values (for numerical solver)
    q2_max  = pi*(syn.r2m);
    

    % be advised, vpasolve ignores all assumptions...
    invkin_eqns = simplify(subs([predyn.forw_q_pos(1:2) == pos_input],syn));
    % disp(invkin_eqns)
    % disp(syn)
    syms q1 q2
    q = [q1;q2];
    q_sol   = vpasolve(invkin_eqns,[q1,q2],[0,q1_max;-q2_max*0.5,q2_max]);
    q_syn   = [q_sol.q1;q_sol.q2];
    
    
    % q sets the rest of the variables
    a_syn   = simplify(subs(subs(predyn.inv0_q_a  ,syn),q,q_syn)); %motor position in rad
    th_syn  = simplify(subs(subs(predyn.forw1_q_th,syn),q,q_syn));
    pos_syn = simplify(subs(subs(predyn.forw_q_pos,syn),q,q_syn)); %verify loop closure & get theta
    
    % disp(pos_syn)
    
    [syn.a1,syn.a2] = deal(a_syn(1),a_syn(2));
    [syn.q1,syn.q2] = deal(q_syn(1),q_syn(2));
    [syn.th1,syn.th2,syn.th3] = deal(th_syn(1),th_syn(2),th_syn(3));
    [syn.x,syn.y,syn.theta] = deal(pos_syn(1),pos_syn(2),pos_syn(3));
    
    % disp(simplify(subs(pos,var_symbols,var_values)))
    
    % substitute constraints / obtain full dynamics
    dyn.J_fing         = double(simplify(subs(predyn.J_fing,syn)));
    dyn.J_full         = double(simplify(subs(predyn.J_full,syn)));
    dyn.id12_fxfy_tauq = simplify(subs(predyn.id12_fxfy_tauq,syn));
    dyn.id_fxfy_taua   = simplify(subs(predyn.id_fxfy_taua,syn));
    dyn.fd_taua_fxfy   = simplify(subs(predyn.fd_taua_fxfy,syn));
    dyn.forw_joint_pos = simplify(subs(predyn.forw_joint_pos,syn));

end

