
% syms_syn = {};
% vals_syn  = {};
syn = struct();

CONCEPT   = 'B-';
ACTUATION = "BALLSCREW"; %available: BALLSCREW, PULLEY

%% Pre-setting constants
% putting in constants

% Link Dimensions
[syn.l1,syn.l2,syn.l3] = deal(50e-3,35e-3,25e-3);

% Pulley Dimensions
[syn.r1m,syn.r1p,syn.r1d] = deal(5e-3,10e-3,7.5e-3);
[syn.r2m] = deal(12.5e-3);
[syn.rxp,syn.rxd] = deal(4.5e-3,5.5e-3);
[syn.rym,syn.ryp] = deal(12e-3,6e-3);

% Antagonistic Properties
[syn.Tyi,syn.ky] = deal(5,300);

% coupler non-deformation constraint
syn.delx = 0;

% Actuator Characteristics
syms taua1_max taua2_max 
syms taua1_stall taua2_stall %stall torque
syms a1dot_max a2dot_max     %no-load speed (rad/s)

if(ACTUATION == "BALLSCREW")
    % Using (conservative) estimates for ballscrew transmission
    [syn.lead1,syn.lead2,syn.eff1,syn.eff2] = deal(1e-3,1e-3,0.9,0.9);

    % ECX-SPEEDP 13M HIGH-POWER version
    [syn.taua1_max,syn.taua1_stall,syn.a1dot_max] = deal(5.93e-3,81.3e-3,66800*2*pi/60);
    [syn.taua2_max,syn.taua2_stall,syn.a2dot_max] = deal(5.93e-3,81.3e-3,66800*2*pi/60);

elseif(ACTUATION == "PULLEY")
    % TODO
end

if CONCEPT(2) == '-'
    syn.rev_v_decay = 0.5; % assume that reversing speed is slowed by this factor
else
    syn.rev_v_decay = 1;
end

predyn_B = tendon_symbolic('B',syn); %the "preliminary" dynamics (unfinished, lacks config data)
predyn_C = tendon_symbolic('C',syn);

%% PRE-SETTING Style
style = struct();

[style.vScale,style.fScale,style.pScale] = deal(5e-3,1e-3,1e-3);
[style.vColor,style.fColor,style.pColor] = deal([0.6350 0.0780 0.1840],[0 0.4470 0.7410],[0.4940 0.1840 0.5560]);

%%

if CONCEPT(1) == 'B'
    predyn = predyn_B;
else
    predyn = predyn_C;
end

% pos_input = [34;104]*1e-3; % around singularity (full ext)
% pos_input = [50;70]*1e-3;
% pos_input = [60;60]*1e-3;
% pos_input = [60;25]*1e-3;
% pos_input = [60;0]*1e-3;
% pos_input = [50;-10]*1e-3;
% pos_input = [10;-20]*1e-3; %around full flexion

pos_inputs = [[34;104] [65;80] [70;20] [10;-20]] * 1e-3;
dyns = [];

for i = 1:numel(pos_inputs(1,:))
    dyns = [dyns get_dyn(predyn,syn,pos_inputs(:,i))];
end


fig = figure('Name', 'Finger Dynamics GUI');
ax = axes('Parent', fig);
set(ax.Title,'String',"CONCEPT ["+CONCEPT+"]");
pbaspect(ax,[2 1 1])
daspect(ax,[1 1 1])

for i = 1:numel(pos_inputs(1,:))
    flower_plotter(CONCEPT,ax,dyns(i),syn,style);
end
