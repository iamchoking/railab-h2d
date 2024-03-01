%% initializing syn values
syn = struct();
% [syn.concept,syn.actuation] = deal(1,1);
[syn.concept,syn.actuation] = deal(2,1);
% concept: 1: B -1: B (closed loop) 2: C -2: C (closed loop)
% actuation: 1: ballscrew

% Link Dimensions
[syn.l1,syn.l2,syn.l3] = deal(45e-3,30e-3,15e-3);
% [syn.l1,syn.l2,syn.l3] = deal(50e-3,35e-3,25e-3);

% Pulley Dimensions
[syn.r1m,syn.r1p,syn.r1d] = deal(5e-3,10e-3,7.5e-3);
[syn.r2m] = deal(12.5e-3);
[syn.rxp,syn.rxd] = deal(4.5e-3,5.5e-3);
[syn.rym,syn.ryp] = deal(12e-3,6e-3);

% Antagonistic Properties
[syn.Tyi,syn.ky] = deal(5,300);

% coupler non-deformation constraint
syn.delx = 0;

if(syn.actuation == 1)
    % Using (conservative) estimates for ballscrew transmission
    [syn.lead1,syn.lead2,syn.eff1,syn.eff2] = deal(0.5e-3,0.5e-3,0.9,0.9);

    % ECX-SPEEDP 13M HIGH-POWER version
    [syn.taua1_max,syn.taua1_stall,syn.a1dot_max] = deal(5.93e-3,81.3e-3,66800*2*pi/60);
    [syn.taua2_max,syn.taua2_stall,syn.a2dot_max] = deal(5.93e-3,81.3e-3,66800*2*pi/60);

elseif(syn.actuation == 2)
    % TODO
end

if syn.concept > 0
    syn.rev_v_decay = 0.5; % assume that reversing speed is slowed by this factor
else
    syn.rev_v_decay = 1;
end

%% PRE-SETTING evaluation positions
% pts = [[0;110] [34;104] [65;80] [70;20] [10;-20]] * 1e-3;
% input_mode = "pos";
% pts = [[0;90] [30;80] [50;60] [65;10] [50;-10]] * 1e-3;

input_mode = "th";
pts = deg2rad([5;11;9]);
pts = [pts deg2rad([linspace(0,90,5);linspace(0,110,5);linspace(0,90,5)])];

%% Initialize style
style = struct();

[style.vScale,style.fScale,style.pScale] = deal(5e-3,1e-3,1e-3);
[style.vColor,style.fColor,style.pColor] = deal([0.6350 0.0780 0.1840],[0 0.4470 0.7410],[0.4940 0.1840 0.5560]);
[style.vVis  ,style.fVis  ,style.pVis  ] = deal(true,true,true);
style.aVis                               = true;

[style.plot_lim,style.setup_lim] = deal([[-40 150];[-40 120]]*1e-3,[[-25 25];[-25 120]]*1e-3);