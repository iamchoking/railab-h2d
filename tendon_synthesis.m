%% Pre-setting constants
% putting in constants
syn = struct();

% [syn.concept,syn.actuation] = deal(1,1);
[syn.concept,syn.actuation] = deal(2,1);
% concept: 1: B -1: B (closed loop) 2: C -2: C (closed loop)
% actuation: 1: ballscrew

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

if(syn.actuation == 1)
    % Using (conservative) estimates for ballscrew transmission
    [syn.lead1,syn.lead2,syn.eff1,syn.eff2] = deal(1e-3,1e-3,0.9,0.9);

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

%% Designating variables (must be a part of "syn")
var_names   = ["r1m" "r1p" "taua1_max"];
var_min     = [5     5     5e-3];
var_max     = [15    10    10e-3];
var_default = [5.1   8     5.93e-3];

for i = 1:length(var_names)
    syn.(var_names(i)) = var_default(i);
end


%% PRE-SETTING Style
style = struct();

[style.vScale,style.fScale,style.pScale] = deal(5e-3,1e-3,1e-3);
[style.vColor,style.fColor,style.pColor] = deal([0.6350 0.0780 0.1840],[0 0.4470 0.7410],[0.4940 0.1840 0.5560]);

[style.plot_lim,style.setup_lim] = deal([[-40 150];[-40 120]]*1e-3,[[-25 25];[-25 120]]*1e-3);

%% Preliminary Calculations (run only once per session) (takes long)

% must remove [var_names] from the pre-calculation (need to be symbolic)
presyn = syn;
for i = 1:length(var_names)
    presyn = rmfield(presyn,var_names(i));
end

predyn_B = tendon_symbolic(presyn,1); %the "preliminary" dynamics (unfinished, lacks config data)
predyn_C = tendon_symbolic(presyn,2);

%% generate ui

fig = figure('Name', 'Finger Dynamics GUI');

% setup axes (shows setup)
ax_setup = axes('Parent', fig,'Position',[0.1, 0.3, 0.3, 0.6]);
xlim(ax_setup,style.setup_lim(1,:));
ylim(ax_setup,style.setup_lim(2,:));
set(ax_setup,'xticklabel',[])
set(ax_setup,'yticklabel',[])
daspect(ax_setup,[1 1 1])

% plot axes (shows flowers, etc)
ax_plot = axes('Parent', fig,'Position',[0.4,0.3,0.6,0.6]);
% set(ax_plot.Title,'String',"CONCEPT ["+syn.concept+"]");
% pbaspect(ax_plot,[2 1 1])
xlim(ax_plot,style.plot_lim(1,:));
ylim(ax_plot,style.plot_lim(2,:));
set(ax_plot,'xticklabel',[])
set(ax_plot,'yticklabel',[])
set(ax_plot,'xtick',style.plot_lim(1,1):0.005:style.plot_lim(1,2))
set(ax_plot,'ytick',style.plot_lim(2,1):0.005:style.plot_lim(2,2))
grid(ax_plot,'on');
daspect(ax_plot,[1 1 1])

% ui: sliders
% gearSlider = uicontrol('Style', 'slider', 'Min', 0, 'Max', 10, ...
%     'Value', gear_ratio, 'Position', [150, 200, 300, 20], 'Callback', @updatePlot);
% pulleySlider = uicontrol('Style', 'slider', 'Min', 0, 'Max', 10, ...
%     'Value', pulley_diameter, 'Position', [150, 170, 300, 20], 'Callback', @updatePlot);
ui_input = struct();
for i=1:length(var_names)
    ui_input.var_names(i) = uicontrol('Style','slider','Min',var_min(i),'Max',var_max(i),'Value',syn.(var_names(i)),'Position',[20 100-20*i 150 20],'Callback',@updatesyn);
end

%% show setup
show_setup(ax_setup,syn,style);

%% get full dynamics from input positions

if abs(syn.concept) == 1
    predyn = predyn_B;
else
    predyn = predyn_C;
end

pos_inputs = [[34;104] [65;80] [70;20] [10;-20]] * 1e-3;
dyns = [];
for i = 1:numel(pos_inputs(1,:))
    dyns = [dyns get_dyn(predyn,syn,pos_inputs(:,i))];
end

%% Calculate and plot
for i = 1:numel(pos_inputs(1,:))
    flower_plotter(ax_plot,dyns(i),syn,style);
end

function updatesyn(control,action)
    disp(get(control))
    disp(get(action))
    for i=1:length(var_names)
        syn.(var_names(i)) = get(ui_input.(var_names(i)),'Value');
    end

    show_setup(ax_setup,syn,style);
end
