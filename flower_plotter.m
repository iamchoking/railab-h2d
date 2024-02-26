function flower_plotter (CONCEPT,ax,dyn,syn)

    REV_V_DECAY    = 0.5; %TODO move this

    VELOCITY_SCALE = 5e-3; % 1m/s is drawn as VELOCITY_SCALEm in the plot
    FORCE_SCALE    = 1e-3;   % 1N is drawn as FORCE_SCALEm in the plot
    POWER_SCALE    = 1e-3;

    if nargin < 1
        % TODO
    end

    %% [STEP 3] Get representative calculations
    
    % Equation for calculating max. force / velocity given the direction in
    % task space (used later for power calculation)
    syms theta_d % prescribed direction placeholder
    unit_d = [cos(theta_d);sin(theta_d)];
    
    % for velocity flower (for any given direction theta_d)
    unit_q     = inv(dyn.J_full)*unit_d; %q1/q2 required to produce unit_d velocity

    maxv_case1 = max([syn.a1dot_max/unit_q(1), -REV_V_DECAY*syn.a1dot_max/unit_q(1)]);
    maxv_case2 = max([syn.a2dot_max/unit_q(2), -REV_V_DECAY*syn.a2dot_max/unit_q(2)]);
    maxv_d_syn = subs(min([maxv_case1 ;maxv_case2]),syn); %(symbolic) maximum speed toward given direction
    
    % for force flower (interplolate btw. 4 vertices)
    % force flowers are harder because input tensions cannot be negative! (+have an offset(antagonistic force))
    syms maxf_d
    syms taua1 taua2
    taua = [taua1;taua2];
    directed_id_eqn_syn = subs(dyn.id_fxfy_taua,{'fx','fy'},{maxf_d*unit_d(1),maxf_d*unit_d(2)}) == taua;
    % directed_fd_eqn_syn = dyn.fd_taua_fxfy == maxf_d*unit_d;
    
    if (CONCEPT(2) == '-')
        case1max_sol   = solve(subs(directed_id_eqn_syn,taua1,syn.taua1_max),[maxf_d,taua2]);
        case1zero_sol  = solve(subs(directed_id_eqn_syn,taua1,0),[maxf_d,taua2]);
        % since these 2 cases are parallel, one of the maxf_d solutions have to
        % be negative!, therefore it is filtered out by symbolic max.
    
        case2max_sol  = solve(subs(directed_id_eqn_syn,taua2,syn.taua2_max),[maxf_d,taua1]);
        case2zero_sol = solve(subs(directed_id_eqn_syn,taua2,0),[maxf_d,taua1]);
        
        maxf_d_syn = simplify(subs(min([max([case1max_sol.maxf_d,case1zero_sol.maxf_d]),max([case2max_sol.maxf_d,case2zero_sol.maxf_d])]),syn));

    elseif(CONCEPT(2) == '+')
        % TODO
    end

    % force flower vertecies
    fant_vec   = subs(dyn.fd_taua_fxfy,taua,[0;0]); %antagonistic force, in task space
    % f0_syn   = subs(dyn.fd_taua_fxfy,taua,[0;0]); %antagonistic force, in task space
    % f1_syn   = subs(dyn.fd_taua_fxfy,taua,[syn.taua1_max;0]);
    % f2_syn   = subs(dyn.fd_taua_fxfy,taua,[0;syn.taua2_max]);
    % fmax_syn = subs(dyn.fd_taua_fxfy,taua,[syn.taua1_max;syn.taua2_max]);


    %% [STEP 4] Start Plotting Data
    
    % Calculate Plot Points!

    % theta_resoluton = deg2rad(10);
    num_points = 10*4;

    
    % finger body points
    plot_origin = [0;0];    
    joint_points = subs(dyn.forw_joint_pos,syn);
    [plot_p1,plot_p2,plot_p3] = deal(joint_points(:,1),joint_points(:,2),joint_points(:,3));
    % plot_p1     = plot_origin+[l1*s1;l1*c1]; % base -> PIP
    % plot_p2     = plot_p1+[l2*s12;l2*c12];   % PIP -> DIP
    % plot_p3     = plot_p2+[l3*s123;l3*c123]; % DIP -> Fingertip ( = pos_syn(1:2))
    % plot_points = [plot_origin plot_p1 plot_p2 plot_p3];
    plot_points = [plot_origin plot_origin+joint_points];

    if(CONCEPT(2) == "-")
        v_inputs = border_inputs(syn.a1dot_max,-REV_V_DECAY * syn.a1dot_max,syn.a2dot_max,-REV_V_DECAY * syn.a2dot_max,num_points);
        f_inputs = border_inputs(syn.taua1_max,0,syn.taua2_max,0,num_points);
    elseif(CONCEPT(2) == "+")
        v_inputs = border_inputs(syn.a1dot_max,-syn.a1dot_max,syn.a2dot_max,-syn.a2dot_max,num_points);
        f_inputs = border_inputs(syn.taua1_max,-syn.taua1_max,syn.taua2_max,-syn.taua2_max,num_points);
    end

    v_borders = dyn.J_full*v_inputs; %resulting border velocities

    f_borders = sym(zeros(2,num_points));
    for(i = 1:length(f_inputs))
        f_borders(:,i) = subs(dyn.fd_taua_fxfy,taua,f_inputs(:,i));
    end %resulting border forces

    p_borders = zeros(2,num_points*2);
    angles    = zeros(1,num_points*2);

    [maxv_ang,maxv_v,maxv_f,maxv_p] = deal(0,0,0,0);
    [maxf_ang,maxf_v,maxf_f,maxf_p] = deal(0,0,0,0);
    [maxp_ang,maxp_v,maxp_f,maxp_p] = deal(0,0,0,0);

    for i_v = 1:num_points
        angle = atan2(v_borders(2,i_v),v_borders(1,i_v));
        unit = subs(unit_d,theta_d,angle);

        angle_v = norm(v_borders(:,i_v));
        angle_f = subs(maxf_d_syn,theta_d,angle);

        angle_p = angle_v*angle_f;

        p_borders(:,i_v)    = unit*angle_p;
        angles(i_v)         = angle;
        % f_borders(:,i_v+num_points) = unit*angle_f;

        if angle_p > maxp_p
            maxp_p = angle_p;
            maxp_ang = angle;
            maxp_v = angle_v;
            maxp_f = angle_f;
        end
        if angle_v > maxv_v
            maxv_v = angle_v;
            maxv_ang = angle;
            maxv_p = angle_p;
            maxv_f = angle_f;
        end

        % angle_f
    end

    for i_f = 1:num_points
        angle = atan2(f_borders(2,i_f),f_borders(1,i_f));
        unit = subs(unit_d,theta_d,angle);

        angle_v = subs(maxv_d_syn,theta_d,angle);
        angle_f = norm(f_borders(:,i_f));

        angle_p = angle_v*angle_f;

        p_borders(:,i_f+num_points)    = unit*angle_p;
        angles(i_f + num_points)       = angle;
        % v_borders(:,i_f+num_points)    = unit*angle_v;
        % disp(power_points(:,i_f+num_points))

        if angle_p > maxp_p
            maxp_p = angle_p;
            maxp_ang = angle;
            maxp_v = angle_v;
            maxp_f = angle_f;
        end

        if angle_f > maxf_f
            maxf_f = angle_f;
            maxf_ang = angle;
            maxf_p  = angle_p;
            maxf_v  = angle_v;
        end
    end

    [~,p_asc_idx] = sort(angles);
    p_borders = p_borders(:,p_asc_idx);

    %% Actual Plotting!
    hold on
    grid on
    % plot the manipulator as a set of lines (TODO: and circles)
    plot(ax,[plot_points(1,1:end-1);plot_points(1,2:end)],[plot_points(2,1:end-1);plot_points(2,2:end)],"cyan",'LineWidth',3)

    % flowers
    % scale into "~_points"
    
    velocity_points = VELOCITY_SCALE*[v_borders v_borders(:,1)] + plot_points(:,end);
    force_points    = FORCE_SCALE   *[f_borders f_borders(:,1)] + plot_points(:,end);
    power_points    = POWER_SCALE   *[p_borders p_borders(:,1)] + plot_points(:,end);
    
    plot(velocity_points(1,:),velocity_points(2,:),"red")
    plot(force_points(1,:),force_points(2,:),"green")
    plot(power_points(1,:),power_points(2,:),"blue")
    % power_points
    
    % TODO
    % annotation for max p/v/f stats
    % maxv
    maxv_unit = [cos(maxv_ang);sin(maxv_ang)]
    maxv_points = [plot_points(:,end) VELOCITY_SCALE*maxv_v*maxv_unit + plot_points(:,end) FORCE_SCALE*maxv_f*maxv_unit + plot_points(:,end) POWER_SCALE*maxv_p*maxv_unit + plot_points(:,end)];
    plot(maxv_points(1,:),maxv_points(2,:),"red");
    scatter(maxv_points(1,2:end),maxv_points(2,2:end),"red");
    text(maxv_points(1,2),maxv_points(2,2),"\leftarrow" + sprintf("v_{maxv}: %0.3f m/s" ,maxv_v),'Color','red');
    text(maxv_points(1,3),maxv_points(2,3),"\leftarrow" + sprintf("f_{maxv}: %0.3f N"   ,maxv_f),'Color','green');
    maxv_flag = text(maxv_points(1,4),maxv_points(2,4),"\leftarrow" + sprintf("p_{maxv}: %0.3f Nm/s",maxv_p),'Color','blue');

    % maxf
    maxf_unit = [cos(maxf_ang);sin(maxf_ang)]
    maxf_points = [plot_points(:,end) VELOCITY_SCALE*maxf_v*maxf_unit + plot_points(:,end) FORCE_SCALE*maxf_f*maxf_unit + plot_points(:,end) POWER_SCALE*maxf_p*maxf_unit + plot_points(:,end)];
    plot(maxf_points(1,:),maxf_points(2,:),"green");
    scatter(maxf_points(1,2:end),maxf_points(2,2:end),"green");
    text(maxf_points(1,2),maxf_points(2,2),"\leftarrow" + sprintf("v_{maxf}: %0.3f m/s" ,maxf_v),'Color','red');
    text(maxf_points(1,3),maxf_points(2,3),"\leftarrow" + sprintf("f_{maxf}: %0.3f N"   ,maxf_f),'Color','green');
    maxf_flag = text(maxf_points(1,4),maxf_points(2,4),"\leftarrow" + sprintf("p_{maxf}: %0.3f Nm/s",maxf_p),'Color','blue');


    % maxp
    maxp_unit = [cos(maxp_ang);sin(maxp_ang)]
    maxp_points = [plot_points(:,end) VELOCITY_SCALE*maxp_v*maxp_unit + plot_points(:,end) FORCE_SCALE*maxp_f*maxp_unit + plot_points(:,end) POWER_SCALE*maxp_p*maxp_unit + plot_points(:,end)];

    if(maxp_ang == maxv_ang || maxp_ang == maxf_ang)
        
    else
        plot(maxp_points(1,:),maxp_points(2,:),"blue");
        scatter(maxp_points(1,2:end),maxp_points(2,2:end),"blue");
        text(maxp_points(1,2),maxp_points(2,2),"\leftarrow" + sprintf("v_{maxp}: %0.3f m/s" ,maxp_v),'Color','red');
        text(maxp_points(1,3),maxp_points(2,3),"\leftarrow" + sprintf("f_{maxp}: %0.3f N"   ,maxp_f),'Color','green');
        text(maxp_points(1,4),maxp_points(2,4),"\leftarrow" + sprintf("p_{maxp}: %0.3f Nm/s",maxp_p),'Color','blue');
    end

    text(maxp_points(1,4),maxp_points(2,4),"Max P \rightarrow",'Color','blue','HorizontalAlignment','right',"FontWeight","bold");


    % add annotation for antagonistic force (and its percentage wrt max force)
    fant_points = [plot_points(:,end) FORCE_SCALE*fant_vec + plot_points(:,end)];
    plot(fant_points(1,:),fant_points(2,:),"black");
    text(fant_points(1,2),fant_points(2,2),"\leftarrow" + sprintf("f_{ant}: %0.3f N"   ,norm(fant_vec)),'Color','black')
    
    % plot_vec = FORCE_SCALE*subs([f0_syn f1_syn f2_syn fmax_syn],syn);
    % plot_ox = plot_points(1,end)*ones(1,size(plot_vec,2));
    % plot_oy = plot_points(2,end)*ones(1,size(plot_vec,2));
    % if(CONCEPT(2) == '+')
    %     plot(ax,[plot_ox; plot_vec(1,:)+plot_ox],[plot_oy; plot_vec(2,:)+plot_oy],'green');
    % elseif(CONCEPT(2) == '-')
    %     plot(ax,[plot_ox(2:end); plot_vec(1,2:end)+plot_ox(2:end)],[plot_oy(2:end); plot_vec(2,2:end)+plot_oy(2:end)],'green');
    %     plot(ax,[plot_ox(1);plot_vec(1,1)+plot_ox(1)],[plot_oy(1);plot_vec(2,1)+plot_oy(1)],'black');
    % end
    % pbaspect(ax,[1 1 1])
    % daspect(ax,[1 1 1])
    % hold off
end
