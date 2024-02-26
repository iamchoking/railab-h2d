
% syms_syn = {};
% vals_syn  = {};
syn = struct();

CONCEPT   = 'C-';
ACTUATION = "BALLSCREW"; %available: BALLSCREW, PULLEY

% putting in constants

[syn.l1,syn.l2,syn.l3] = deal(50e-3,35e-3,25e-3);
% syms_syn = [syms_syn {l1    l2    l3   }];
% vals_syn = [vals_syn {50e-3 35e-3 25e-3}];

[syn.r1m,syn.r1p,syn.r1d] = deal(5e-3,10e-3,7.5e-3);
% syms_syn = [syms_syn {r1m     r1p      r1d  }];
% vals_syn = [vals_syn {5e-3    10e-3  7.5e-3 }];
[syn.r2m] = deal(12.5e-3);
% syms_syn = [syms_syn {r2m                   }];
% vals_syn = [vals_syn {12.5e-3               }];
[syn.rxp,syn.rxd] = deal(4.5e-3,5.5e-3);
% syms_syn = [syms_syn {        rxp     rxd   }];
% vals_syn = [vals_syn {        4.5e-3  5.5e-3}];

[syn.rym,syn.ryp] = deal(12e-3,6e-3);
% syms_syn = [syms_syn {rym     ryp           }];
% vals_syn = [vals_syn {12e-3   6e-3          }]; % rym only used by concept B

% syms_syn = [syms_syn {r1m     r1p      r1d  }];
% vals_syn = [vals_syn {5e-3    9e-3     6e-3 }];
% syms_syn = [syms_syn {r2m                   }];
% vals_syn = [vals_syn {20e-3                 }];
% syms_syn = [syms_syn {        rxp     rxd   }];
% vals_syn = [vals_syn {        4.5e-3  5.5e-3}];
% syms_syn = [syms_syn {rym     ryp           }];
% vals_syn = [vals_syn {13.5e-3 11e-3          }]; % rym only used by concept B

[syn.Tyi,syn.ky] = deal(5,300);
% syms_syn = [syms_syn {Tyi  ky  }];
% vals_syn = [vals_syn  {  5  300 }];

% coupler non-deformation constraint
syn.delx = 0;
% assume(delx == 0)

% Actuation Characteristics
syms taua1_max taua2_max 
syms taua1_stall taua2_stall %stall torque
syms a1dot_max a2dot_max     %no-load speed (rad/s)

% adot_max = [a1dot_max;a2dot_max];
% taua_max = [taua1_max;taua2_max];

if(ACTUATION == "BALLSCREW")
    % Using (conservative) estimates for ballscrew transmission
    [syn.lead1,syn.lead2,syn.eff1,syn.eff2] = deal(1e-3,1e-3,0.9,0.9);
    % syms_syn = [syms_syn  {lead1 lead2 eff1  eff2}];
    % vals_syn = [vals_syn  {1e-3  1e-3  0.9   0.9 }];

    % ECX-SPEEDP 13M HIGH-POWER version
    [syn.taua1_max,syn.taua1_stall,syn.a1dot_max] = deal(5.93e-3,81.3e-3,66800*2*pi/60);
    [syn.taua2_max,syn.taua2_stall,syn.a2dot_max] = deal(5.93e-3,81.3e-3,66800*2*pi/60);
    % syms_syn = [syms_syn  {taua1_max taua2_max taua1_stall taua2_stall a1dot_max     a2dot_max     }];
    % vals_syn = [vals_syn  {5.93e-3   5.93e-3   81.3e-3     81.3e-3     66800*2*pi/60 66800*2*pi/60 }];

elseif(ACTUATION == "PULLEY")
    % TODO
end


predyn = tendon_symbolic(CONCEPT,syn); %the "preliminary" dynamics (unfinished, lacks config data)

%% [STEP 0] Establish Configuration (pos,q,a,th)

% Approach 1: set configuration via inverse kinematics
% <INPUT> Desired position
% pos_input = [34;104]*1e-3; % around singularity (full ext)
% pos_input = [50;70]*1e-3;
% pos_input = [60;60]*1e-3;
% pos_input = [60;25]*1e-3;
% pos_input = [60;0]*1e-3;
% pos_input = [50;-10]*1e-3;
pos_input = [10;-20]*1e-3; %around full flexion

dyn = get_dyn(predyn,syn,pos_input);

% % <OUTPUT> q, th, a values from Numerical Inverse Kinematics
% q1_max  = pi*(syn.r1m + syn.r1p + syn.r1d); %set crude max/min values (for numerical solver)
% q2_max  = pi*(syn.r2m);
% % q1_max  = subs(pi*(r1m + r1p + r1d),syn); %set crude max/min values (for numerical solver)
% % q2_max  = subs(pi*(r2m),syn);
% 
% % be advised, vpasolve ignores all assumptions...
% invkin_eqns = [predyn.forw_q_pos(1:2) == pos_input];
% syms q1 q2
% q = [q1;q2];
% q_sol   = vpasolve(invkin_eqns,[q1,q2],[0,q1_max;-q2_max*0.5,q2_max]);
% q_syn   = [q_sol.q1;q_sol.q2];
% 
% 
% % q sets the rest of the variables
% a_syn   = simplify(subs(predyn.inv0_q_a  ,q,q_syn)); %motor position in rad
% th_syn  = simplify(subs(predyn.forw1_q_th,q,q_syn));
% pos_syn = subs(predyn.forw_q_pos,q,q_syn); %verify loop closure & get theta
% 
% % disp(pos_syn)
% 
% %% [STEP 2] Get Dynamics wrt calculated position data
% 
% [syn.q1,syn.q2] = deal(q_syn(1),q_syn(2));
% % syms_syn  = [syms_syn {q1       q2       }];
% % vals_syn  = [vals_syn {q_syn(1) q_syn(2) }];
% [syn.th1,syn.th2,syn.th3] = deal(th_syn(1),th_syn(2),th_syn(3));
% % syms_syn  = [syms_syn {th1       th2       th3       }];
% % vals_syn  = [vals_syn {th_syn(1) th_syn(2) th_syn(3) }];
% [syn.x,syn.y,syn.theta] = deal(pos_syn(1),pos_syn(2),pos_syn(3));
% % syms_syn  = [syms_syn {x          y          theta      }];
% % vals_syn  = [vals_syn {pos_syn(1) pos_syn(2) pos_syn(3) }];
% 
% % disp(simplify(subs(pos,var_symbols,var_values)))
% 
% % substitute constraints / obtain predynamics
% J_fing_syn       = simplify(subs(predyn.J_fing,syn));
% J_full_syn       = simplify(subs(predyn.J_full,syn));
% 
% id12_fxfy_tauq_syn = simplify(subs(predyn.id12_fxfy_tauq,syn));
% 
% id_fxfy_taua_syn = simplify(subs(predyn.id_fxfy_taua,syn));
% fd_taua_fxfy_syn = simplify(subs(predyn.fd_taua_fxfy,syn));

%% actual calc / drawing

% auto-plot
fig = figure('Name', 'Robot predynamics GUI');
ax = axes('Parent', fig);
flower_plotter(CONCEPT,ax,dyn,syn);

%% [STEP 3] Get representative calculations

% % Equation for calculating max. force / velocity given the direction in
% % task space (used later for power calculation)
% syms theta_d % prescribed direction
% unit_d = [cos(theta_d);sin(theta_d)];
% 
% % for velocity flower (for any given direction theta_d)
% unit_q     = inv(J_fing_syn)*unit_d; %q1/q2 required to produce unit_d velocity
% maxv_d_syn = subs(min([a1dot_max/abs(unit_q(1));a2dot_max/abs(unit_q(2))]),syn); %(symbolic) maximum speed toward given direction
% 
% % spd_sign = solve(J_fing_syn*adot == unit_d*d_speed,adot);
% % spd_sign1 = sign(diff(spd_sign.a1dot,d_speed)); %must increase a1dot to increase d_speed
% % spd_sign2 = sign(diff(spd_sign.a2dot,d_speed));
% % 
% % spd_case1 = solve(subs(J_fing_syn*adot == [cos(theta_d);sin(theta_d)]*d_speed,a1dot,a1dot_max*spd_sign1),a2dot,d_speed);
% % spd_case2 = solve(subs(J_fing_syn*adot == [cos(theta_d);sin(theta_d)]*d_speed,a2dot,a2dot_max*spd_sign2),a1dot,d_speed);
% 
% % for force flower (interplolate btw. 4 vertices)
% % force flowers are harder because input tensions cannot be negative!
% % (+have an offset(antagonistic force))
% 
% syms maxf_d
% directed_id_eqn_syn = subs(id_fxfy_taua_syn,f(1:2),maxf_d*unit_d) == taua;
% directed_fd_eqn_syn = fd_taua_fxfy_syn == maxf_d*unit_d;
% 
% 
% if (CONCEPT(2:2) == '-')
%     case1max_sol   = solve(subs(directed_id_eqn_syn,taua1,taua1_max),[maxf_d,taua2]);
%     % case1max_sol2  =
%     % solve(subs(directed_fd_eqn_syn,taua1,taua1_max),[maxf_d,taua2]);
%     case1zero_sol = solve(subs(directed_id_eqn_syn,taua1,0),[maxf_d,taua2]);
%     % since these 2 cases are parallel, one of the maxf_d solutions have to
%     % be negative!, therefore filtered out by symbolic max.
% 
%     case2max_sol = solve(subs(directed_id_eqn_syn,taua2,taua2_max),[maxf_d,taua1]);
%     case2zero_sol = solve(subs(directed_id_eqn_syn,taua2,0),[maxf_d,taua1]);
% 
%     maxf_d_syn = simplify(subs(min([max([case1max_sol.maxf_d,case1zero_sol.maxf_d]),max([case2max_sol.maxf_d,case2zero_sol.maxf_d])]),syn));
% 
% elseif(CONCEPT(2:1) == '+')
% end
% 
% f0_syn   = subs(fd_taua_fxfy_syn,taua,[0;0]);
% f1_syn   = subs(fd_taua_fxfy_syn,taua,[taua1_max;0]);
% f2_syn   = subs(fd_taua_fxfy_syn,taua,[0;taua2_max]);
% fmax_syn = subs(fd_taua_fxfy_syn,taua,[taua1_max;taua2_max]);

% disp(fmax_syn);
% disp(f1_syn);

% %% [STEP 4] Start Plotting Data
% 
% % Calculate Plot Points!
% FORCE_SCALE    = 1e-3;   % 1N is drawn as FORCE_SCALEm in the plot
% VELOCITY_SCALE = 1e-6; % 1m/s is drawn as VELOCITY_SCALEm in the plot
% POWER_SCALE    = 1e-7;
% % theta_resoluton = deg2rad(10);
% num_points = 10*4;
% % manipulator config plotting
% 
% %input angles must be determined "smartly", as they need to be "evenly
% %spread" along the jacobian angles for a good representation.
% 
% plot_origin = [0;0];
% 
% plot_p1     = plot_origin+[l1*s1;l1*c1];
% plot_p2     = plot_p1+[l2*s12;l2*c12];
% plot_p3     = plot_p2+[l3*s123;l3*c123]; %( = pos_syn(1:2))
% plot_points = subs([plot_origin plot_p1 plot_p2 plot_p3],syn);
% 
% if(CONCEPT(2) == "-")
%     v_inputs = border_inputs(a1dot_max,-REV_V_DECAY * a1dot_max,a2dot_max,-REV_V_DECAY * a2dot_max,num_points);
%     f_inputs = border_inputs(taua1_max,0,taua2_max,0,num_points);
% elseif(CONCEPT(2) == "+")
%     v_inputs = border_inputs(a1dot_max,-a1dot_max,a2dot_max,-a2dot_max,num_points);
%     f_inputs = border_inputs(taua1_max,-taua1_max,taua2_max,-taua2_max,num_points);
% end
% 
% v_borders = J_fing_syn*v_inputs;
% f_borders = sym(zeros(2,num_points));
% input_angles_v = zeros(num_points);
% input_angles_f = zeros(num_points);
% % f_borders = [];
% for(i = 1:length(f_inputs))
%     f_borders(:,i) = subs(fd_taua_fxfy_syn,taua,f_inputs(:,i));
% end
% 
% % resolve values first (computaiontal)
% v_borders = subs(v_borders,syn);
% f_borders = subs(f_borders,syn);
% 
% power_points = zeros(2,num_points*2);
% 
% for i_v = 1:num_points
%     angle = atan2(v_borders(2,i_v),v_borders(1,i_v));
%     unit = subs(unit_d,theta_d,angle);
% 
%     maxv = norm(v_borders(:,i_v));
%     maxf = subs(maxf_d_syn,theta_d,angle);
% 
%     maxp = maxv*maxf;
% 
%     power_points(:,i_v)    = subs(unit*maxp*POWER_SCALE,syn)    + plot_points(:,end);
% end
% 
% for i_f = 1:num_points
%     angle = atan2(f_borders(2,i_f),f_borders(1,i_f));
%     unit = subs(unit_d,theta_d,angle);
% 
%     maxv = subs(maxv_d_syn,theta_d,angle);
%     maxf = norm(f_borders(:,i_f));
% 
%     maxp = maxv*maxf;
% 
%     power_points(:,i_f+num_points)    = subs(unit*maxp*POWER_SCALE,syn)    + plot_points(:,end);
%     % disp(power_points(:,i_f+num_points))
% end
% 
% 
% 
% force_points    = FORCE_SCALE*subs(f_borders,syn)    + plot_points(:,end);
% velocity_points = VELOCITY_SCALE*subs(v_borders,syn) + plot_points(:,end);
% 
% 
% %% Actual Plotting!
% 
% % plot the manipulator as a set of lines and circles
% plot([plot_points(1,1:end-1);plot_points(1,2:end)],[plot_points(2,1:end-1);plot_points(2,2:end)],"cyan")
% hold on
% grid on
% 
% % plot the force drawing
% % plot_f = simplify(FORCE_SCALE*subs([f0_syn f1_syn f2_syn fmax_syn],syn));
% % plot([pos_syn(1);plot_f(1,1)+pos_syn(1)]  ,[pos_syn(2);plot_f(2,1)+pos_syn(2)]  ,'black');
% % plot([pos_syn(1);plot_f(1,2)+pos_syn(1)]  ,[pos_syn(2);plot_f(2,2)+pos_syn(2)]  ,'blue');
% % plot([pos_syn(1);plot_f(1,3)+pos_syn(1)]  ,[pos_syn(2);plot_f(2,3)+pos_syn(2)]  ,'red');
% % plot([pos_syn(1);plot_f(1,4)+pos_syn(1)]  ,[pos_syn(2);plot_f(2,4)+pos_syn(2)]  ,'green');
% 
% scatter(velocity_points(1,:),velocity_points(2,:),"red")
% scatter(force_points(1,:),force_points(2,:),"green",'x')
% scatter(power_points(1,:),power_points(2,:),"blue")
% 
% plot_vec = FORCE_SCALE*subs([f0_syn f1_syn f2_syn fmax_syn],syn);
% plot_ox = plot_points(1,end)*ones(1,size(plot_vec,2));
% plot_oy = plot_points(2,end)*ones(1,size(plot_vec,2));
% if(CONCEPT(2) == '+')
%     plot([plot_ox; plot_vec(1,:)+plot_ox],[plot_oy; plot_vec(2,:)+plot_oy],'green');
% elseif(CONCEPT(2) == '-')
%     plot([plot_ox(2:end); plot_vec(1,2:end)+plot_ox(2:end)],[plot_oy(2:end); plot_vec(2,2:end)+plot_oy(2:end)],'green');
%     plot([plot_ox(1);plot_vec(1,1)+plot_ox(1)],[plot_oy(1);plot_vec(2,1)+plot_oy(1)],'black');
% end
% pbaspect([1 1 1])
% daspect([1 1 1])
% % hold off
% 
