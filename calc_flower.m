function calc = calc_flower (dyn,syn)

    % if nargin < 1
    % end

    calc.singular = dyn.singular;      

    %% Get representative calculations
    
    calc.maxv = struct();
    calc.maxf = struct();
    calc.maxp = struct();

    if dyn.singular %special treatment for sigular cases
        disp("[CALC] !!Calculating for singular case")
        sing_unit = dyn.J_fing(:,1)/norm(dyn.J_fing(:,1)); %all resulting velocities are parallel to this vector.
        sing_ang  = atan2(sing_unit(2),sing_unit(1));
        syms sing_vmag sing_fmag;
        syms taua1 taua2;
        syms fx fy;

        disp("[CALC] Singular axis: ["+strjoin(string(sing_unit))+"] (angle " + rad2deg(sing_ang) + "deg)");
        

        sing_id_fxfy_taua = subs(dyn.id_fxfy_taua,{'fx','fy'},{sing_unit(1)*sing_fmag,sing_unit(2)*sing_fmag});
        sing_id12_fxfy_tauq = subs(dyn.id12_fxfy_tauq,{'fx','fy'},{sing_unit(1)*sing_fmag,sing_unit(2)*sing_fmag});

        % forward dynamics (input: taua1 -> output: sing_fmag (magnitude of force along singular direction) & required taua2
        sing_fd_taua1_fxfy_sol = solve(sing_id_fxfy_taua==[taua1;taua2],[sing_fmag taua2]);
        sing_fd_taua2_fxfy_sol = solve(sing_id_fxfy_taua==[taua1;taua2],[sing_fmag taua1]);

        calc.fant_vec = sing_unit*subs(sing_fd_taua1_fxfy_sol.sing_fmag,taua1,0); %incomplete solution...

        joint_points = subs(dyn.forw_joint_pos,syn);
        calc.plot_points = double([[0;0] joint_points]);
        
        calc.v_borders = [];
        calc.f_borders = [];
        calc.p_borders = [];
        calc.angles    = [];
        calc.maxv.ang  = sing_ang;
        calc.maxv.v    = max(transpose(sing_unit)*dyn.J_full(:,1)*syn.a1dot_max,-transpose(sing_unit)*dyn.J_full(:,1)*syn.a1dot_max*syn.rev_v_decay) + max(transpose(sing_unit)*dyn.J_full(:,2)*syn.a2dot_max,-transpose(sing_unit)*dyn.J_full(:,2)*syn.a2dot_max*syn.rev_v_decay);
        calc.maxv.f    = min(subs(sing_fd_taua1_fxfy_sol.sing_fmag,taua1,syn.taua1_max),subs(sing_fd_taua2_fxfy_sol.sing_fmag,taua2,syn.taua2_max));
        calc.maxv.p    = calc.maxv.v*calc.maxv.f;

        calc.maxf  = calc.maxv;
        calc.maxp  = calc.maxv;
    else
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
        joint_points = subs(dyn.forw_joint_pos,syn);
        calc.plot_points = double([[0;0] joint_points]);
    
        if(syn.concept > 0)
            v_inputs = border_inputs(syn.a1dot_max,-syn.rev_v_decay * syn.a1dot_max,syn.a2dot_max,-syn.rev_v_decay * syn.a2dot_max,num_points);
            f_inputs = border_inputs(syn.taua1_max,0,syn.taua2_max,0,num_points);
        elseif(syn.concept < 0)
            v_inputs = border_inputs(syn.a1dot_max,-syn.a1dot_max,syn.a2dot_max,-syn.a2dot_max,num_points);
            f_inputs = border_inputs(syn.taua1_max,-syn.taua1_max,syn.taua2_max,-syn.taua2_max,num_points);
        end
    
        calc.v_borders = double(dyn.J_full*v_inputs); %resulting border velocities
    
        calc.f_borders = zeros(2,num_points);
        for i = 1:length(f_inputs)
            calc.f_borders(:,i) = double(subs(dyn.fd_taua_fxfy,taua,f_inputs(:,i)));
        end %resulting border forces
    
        calc.p_borders = zeros(2,num_points*2);
        calc.angles    = zeros(1,num_points*2);
    
        [calc.maxv.ang,calc.maxv.v,calc.maxv.f,calc.maxv.p] = deal(0,0,0,0);
        [calc.maxf.ang,calc.maxf.v,calc.maxf.f,calc.maxf.p] = deal(0,0,0,0);
        [calc.maxp.ang,calc.maxp.v,calc.maxp.f,calc.maxp.p] = deal(0,0,0,0);
    
        for i_v = 1:num_points
            angle = atan2(calc.v_borders(2,i_v),calc.v_borders(1,i_v));
            unit = subs(unit_d,theta_d,angle);
    
            angle_v = norm(calc.v_borders(:,i_v));
            angle_f = double(subs(maxf_d_syn,theta_d,angle));
    
            angle_p = angle_v*angle_f;
    
            calc.p_borders(:,i_v)    = unit*angle_p;
            calc.angles(i_v)         = angle;
            % calc.f_borders(:,i_v+num_points) = unit*angle_f;
    
            if angle_p > calc.maxp.p
                calc.maxp.p = angle_p;
                calc.maxp.ang = angle;
                calc.maxp.v = angle_v;
                calc.maxp.f = angle_f;
            end
            if angle_v > calc.maxv.v
                calc.maxv.v   = angle_v;
                calc.maxv.ang = angle;
                calc.maxv.p   = angle_p;
                calc.maxv.f   = angle_f;
            end
    
            % angle_f
        end

        for i_f = 1:num_points
            angle = atan2(calc.f_borders(2,i_f),calc.f_borders(1,i_f));
            unit = subs(unit_d,theta_d,angle);
    
            angle_v = double(subs(maxv_d_syn,theta_d,angle));
            angle_f = norm(calc.f_borders(:,i_f));
    
            angle_p = angle_v*angle_f;
    
            calc.p_borders(:,i_f+num_points)    = unit*angle_p;
            calc.angles(i_f + num_points)       = angle;
            % calc.v_borders(:,i_f+num_points)    = unit*angle_v;
            % disp(p_points(:,i_f+num_points))
    
            if angle_p > calc.maxp.p
                calc.maxp.p = angle_p;
                calc.maxp.ang = angle;
                calc.maxp.v = angle_v;
                calc.maxp.f = angle_f;
            end
    
            if angle_f > calc.maxf.f
                calc.maxf.f   = angle_f;
                calc.maxf.ang = angle;
                calc.maxf.p   = angle_p;
                calc.maxf.v   = angle_v;
            end
        end
        [~,p_asc_idx] = sort(calc.angles);
        calc.p_borders = calc.p_borders(:,p_asc_idx);
    end


end
