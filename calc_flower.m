function calc = calc_flower (dyn,syn)

    % if nargin < 1
    % end

    calc.singular    = dyn.singular;      
    calc.q           = dyn.q;
    calc.th          = dyn.th;
    calc.a           = dyn.a;
    calc.pos         = dyn.pos;
    calc.J_fing      = dyn.J_fing;
    calc.T_max       = [syn.taua1_max*(2*pi/syn.lead1)*syn.eff1;syn.taua2_max*(2*pi/syn.lead2)*syn.eff2];
    calc.plot_points = double([[0;0] dyn.fk_joint_pos]);

    %% Get representative calculations
    
    calc.maxv = struct();
    calc.maxf = struct();
    calc.maxp = struct();
    calc.warn = false;

    adot_max = [syn.a1dot_max;syn.a2dot_max];
    taua_max = [syn.taua1_max;syn.taua2_max];
    if(syn.concept > 0)
        adot_min = -syn.rev_v_decay*adot_max;
        taua_min = [0;0];
    else % full reverse actuation for closed-loop variants
        adot_min = -adot_max;
        taua_min = -taua_max;
    end

    if dyn.singular %special treatment for sigular cases
        disp("[CALC] !!Calculating for singular case")
        sing_unit = dyn.J_fing(:,1)/norm(dyn.J_fing(:,1)); %all resulting velocities are parallel to this vector.
        sing_ang  = atan2(sing_unit(2),sing_unit(1));
        % legacy (symbolic) approach is redacted. see previous commits.
        inv_J_full = pinv(dyn.J_full);
        calc.fant_vec = -sing_unit*maxf_d(-sing_ang);
        
        calc.v_borders = [];
        calc.f_borders = [];
        calc.p_borders = [];
        calc.angles    = [];
        calc.maxv.ang  = sing_ang;
        calc.maxv.v    = maxv_d(sing_ang);
        calc.maxv.f    = maxf_d(sing_ang);
        calc.maxv.p    = calc.maxv.v*calc.maxv.f;

        calc.maxf  = calc.maxv;
        calc.maxp  = calc.maxv;
    else
        % function for calculating max. force / velocity given the direction in
        % task space (used later for power calculation)

        inv_J_full = inv(dyn.J_full);
        
        % antagonistic force
        calc.fant_vec   = dyn.f_ant; %antagonistic force, in task space
    
        num_points = 10*4;
        
        if(syn.concept > 0)
            v_inputs = border_inputs(syn.a1dot_max,-syn.rev_v_decay * syn.a1dot_max,syn.a2dot_max,-syn.rev_v_decay * syn.a2dot_max,num_points);
            f_inputs = border_inputs(syn.taua1_max,0,syn.taua2_max,0,num_points);
        elseif(syn.concept < 0)
            v_inputs = border_inputs(syn.a1dot_max,-syn.a1dot_max,syn.a2dot_max,-syn.a2dot_max,num_points);
            f_inputs = border_inputs(syn.taua1_max,-syn.taua1_max,syn.taua2_max,-syn.taua2_max,num_points);
        end
    
        
        calc.v_borders = dyn.J_full*v_inputs; %resulting border velocities
        calc.f_borders = dyn.fd_A*f_inputs + dyn.fd_b;
    
        calc.p_borders = zeros(2,num_points*2);
        calc.angles    = zeros(1,num_points*2);
    
        [calc.maxv.ang,calc.maxv.v,calc.maxv.f,calc.maxv.p] = deal(0,0,0,0);
        [calc.maxf.ang,calc.maxf.v,calc.maxf.f,calc.maxf.p] = deal(0,0,0,0);
        [calc.maxp.ang,calc.maxp.v,calc.maxp.f,calc.maxp.p] = deal(0,0,0,0);
    
        for i_v = 1:num_points
            angle = atan2(calc.v_borders(2,i_v),calc.v_borders(1,i_v));
            unit = [cos(angle);sin(angle)];
    
            angle_v = double(norm(calc.v_borders(:,i_v)));
            angle_f = maxf_d(angle);
    
            angle_p = angle_v*angle_f;
    
            calc.p_borders(:,i_v)    = unit*angle_p;
            calc.angles(i_v)         = angle;
            % calc.f_borders(:,i_v+num_points) = unit*angle_f;
            % calc.f_borders(:,i_v) = unit*angle_f;
    
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
            unit = [cos(angle);sin(angle)];
    
            angle_v = maxv_d(angle);
            angle_f = double(norm(calc.f_borders(:,i_f)));
    
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
    % for depicting antagonistic forces
    calc.zero = struct();
    [calc.zero.ang,calc.zero.v,calc.zero.f,calc.zero.p] = deal(0,0,0,0);
    calc.zero.dely = dyn.dely;

    cats = ["maxv","maxf","maxp","zero"];
    for i = 1:length(cats)
        fx = calc.(cats(i)).f * cos(calc.(cats(i)).ang);
        fy = calc.(cats(i)).f * sin(calc.(cats(i)).ang);
        % calc.(cats(i)).tauq = double(subs(dyn.id12_fxfy_tauq,{"fx","fy"},{fx,fy}));
        calc.(cats(i)).tauq = dyn.id12_A*[fx;fy] + dyn.id12_b;
        % calc.(cats(i)).taux = double(subs(dyn.id_fxfy_taux  ,{"fx","fy"},{fx,fy}));
        calc.(cats(i)).taux = dyn.idx_A*[fx;fy] + dyn.idx_b;
        %TODO: deduce qdot / adot
        
    end

    if min(calc.zero.tauq) <= 0
        disp("[CALC][WARN] Negative Antagonistic Force Detected.")
        calc.warn = true;
    end

    function maxv = maxv_d(ang)
        unit_d = [cos(ang);sin(ang)];
        unit_adot = inv_J_full*unit_d; % adot required to produce unit_d velocity

        case1 = max(adot_max(1)/unit_adot(1),adot_min(1)/unit_adot(1));
        case2 = max(adot_max(2)/unit_adot(2),adot_min(2)/unit_adot(2));

        maxv = min(case1,case2);
        
        if maxv < 0
            disp("[WARN] NEGATIVE maxv_d value!");
        end

    end

    function maxf = maxf_d(ang)
        unit_d = [cos(ang);sin(ang)];
        % taua = id_A * f  + b, where f = maxf*unit_d
        % taua = maxf * (id_A*unit_d) + b 
        %TODO: this returns erroneous
        % values in negative situations!!! (BOOKMARK)
        id_A_d = dyn.id_A*unit_d;

        case1 = max((taua_min(1) - dyn.id_b(1))/id_A_d(1),(taua_max(1) - dyn.id_b(1))/id_A_d(1));
        case2 = max((taua_min(2) - dyn.id_b(2))/id_A_d(2),(taua_max(2) - dyn.id_b(2))/id_A_d(2));
        
        maxf = min(case1,case2);

        if maxf < 0
            disp("[WARN] NEGATIVE maxf_d value!");
        end
    end

end
