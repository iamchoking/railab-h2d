function calc = calc_flower (dyn,syn)

    % if nargin < 1
    % end

    %% Get representative calculations
    
    % Equation for calculating max. force / velocity given the direction in
    % task space (used later for power calculation)
    syms theta_d % prescribed direction placeholder
    unit_d = [cos(theta_d);sin(theta_d)];
    
    % for velocity flower (for any given direction theta_d)
    unit_q     = inv(dyn.J_full)*unit_d; %q1/q2 required to produce unit_d velocity

    maxv_case1 = max([syn.a1dot_max/unit_q(1), -syn.rev_v_decay*syn.a1dot_max/unit_q(1)]);
    maxv_case2 = max([syn.a2dot_max/unit_q(2), -syn.rev_v_decay*syn.a2dot_max/unit_q(2)]);
    maxv_d_syn = subs(min([maxv_case1 ;maxv_case2]),syn); %(symbolic) maximum speed toward given direction
    
    % for force flower (interplolate btw. 4 vertices)
    % force flowers are harder because input tensions cannot be negative! (+have an offset(antagonistic force))
    syms maxf_d
    syms taua1 taua2
    taua = [taua1;taua2];
    directed_id_eqn_syn = subs(dyn.id_fxfy_taua,{'fx','fy'},{maxf_d*unit_d(1),maxf_d*unit_d(2)}) == taua;
    % directed_fd_eqn_syn = dyn.fd_taua_fxfy == maxf_d*unit_d;
    
    if (syn.concept > 0)
        case1max_sol   = solve(subs(directed_id_eqn_syn,taua1,syn.taua1_max),[maxf_d,taua2]);
        case1zero_sol  = solve(subs(directed_id_eqn_syn,taua1,0),[maxf_d,taua2]);
        % since these 2 cases are parallel, one of the maxf_d solutions have to
        % be negative!, therefore it is filtered out by symbolic max.
    
        case2max_sol  = solve(subs(directed_id_eqn_syn,taua2,syn.taua2_max),[maxf_d,taua1]);
        case2zero_sol = solve(subs(directed_id_eqn_syn,taua2,0),[maxf_d,taua1]);
        
        maxf_d_syn = simplify(subs(min([max([case1max_sol.maxf_d,case1zero_sol.maxf_d]),max([case2max_sol.maxf_d,case2zero_sol.maxf_d])]),syn));

    elseif(syn.concept < 0)
        % TODO
    end

    % antagonistic force
    calc.fant_vec   = double(subs(dyn.fd_taua_fxfy,taua,[0;0])); %antagonistic force, in task space

    %% [STEP 4] Calculate Plotting Data
    
    % Calculate Plot Points!

    % theta_resoluton = deg2rad(10);
    num_points = 10*4;
    
    % finger body points
    plot_origin = [0;0];    
    joint_points = subs(dyn.forw_joint_pos,syn);
    calc.plot_points = double([plot_origin plot_origin+joint_points]);

    if(syn.concept > 0)
        v_inputs = border_inputs(syn.a1dot_max,-syn.rev_v_decay * syn.a1dot_max,syn.a2dot_max,-syn.rev_v_decay * syn.a2dot_max,num_points);
        f_inputs = border_inputs(syn.taua1_max,0,syn.taua2_max,0,num_points);
    elseif(syn.concept < 0)
        v_inputs = border_inputs(syn.a1dot_max,-syn.a1dot_max,syn.a2dot_max,-syn.a2dot_max,num_points);
        f_inputs = border_inputs(syn.taua1_max,-syn.taua1_max,syn.taua2_max,-syn.taua2_max,num_points);
    end

    calc.v_borders = double(dyn.J_full*v_inputs); %resulting border velocities

    calc.f_borders = zeros(2,num_points);
    for(i = 1:length(f_inputs))
        calc.f_borders(:,i) = double(subs(dyn.fd_taua_fxfy,taua,f_inputs(:,i)));
    end %resulting border forces

    calc.p_borders = zeros(2,num_points*2);
    calc.angles    = zeros(1,num_points*2);

    [calc.maxv_ang,calc.maxv_v,calc.maxv_f,calc.maxv_p] = deal(0,0,0,0);
    [calc.maxf_ang,calc.maxf_v,calc.maxf_f,calc.maxf_p] = deal(0,0,0,0);
    [calc.maxp_ang,calc.maxp_v,calc.maxp_f,calc.maxp_p] = deal(0,0,0,0);

    for i_v = 1:num_points
        angle = atan2(calc.v_borders(2,i_v),calc.v_borders(1,i_v));
        unit = subs(unit_d,theta_d,angle);

        angle_v = norm(calc.v_borders(:,i_v));
        angle_f = subs(maxf_d_syn,theta_d,angle);

        angle_p = angle_v*angle_f;

        calc.p_borders(:,i_v)    = unit*angle_p;
        calc.angles(i_v)         = angle;
        % calc.f_borders(:,i_v+num_points) = unit*angle_f;

        if angle_p > calc.maxp_p
            calc.maxp_p = angle_p;
            calc.maxp_ang = angle;
            calc.maxp_v = angle_v;
            calc.maxp_f = angle_f;
        end
        if angle_v > calc.maxv_v
            calc.maxv_v = angle_v;
            calc.maxv_ang = angle;
            calc.maxv_p = angle_p;
            calc.maxv_f = angle_f;
        end

        % angle_f
    end

    for i_f = 1:num_points
        angle = atan2(calc.f_borders(2,i_f),calc.f_borders(1,i_f));
        unit = subs(unit_d,theta_d,angle);

        angle_v = subs(maxv_d_syn,theta_d,angle);
        angle_f = norm(calc.f_borders(:,i_f));

        angle_p = angle_v*angle_f;

        calc.p_borders(:,i_f+num_points)    = unit*angle_p;
        calc.angles(i_f + num_points)       = angle;
        % calc.v_borders(:,i_f+num_points)    = unit*angle_v;
        % disp(p_points(:,i_f+num_points))

        if angle_p > calc.maxp_p
            calc.maxp_p = angle_p;
            calc.maxp_ang = angle;
            calc.maxp_v = angle_v;
            calc.maxp_f = angle_f;
        end

        if angle_f > calc.maxf_f
            calc.maxf_f = angle_f;
            calc.maxf_ang = angle;
            calc.maxf_p  = angle_p;
            calc.maxf_v  = angle_v;
        end
    end

    [~,p_asc_idx] = sort(calc.angles);
    calc.p_borders = calc.p_borders(:,p_asc_idx);
end
