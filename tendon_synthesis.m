%% Pre-setting constants
% putting in constants
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

%% Designating variables (must be a part of "syn")
var_names   = ["r1m"    "r1p"   "r1d"   "r2m"   "taua1_max" "taua2_max"];
var_min     = [5*1e-3   5*1e-3  5*1e-3  5*1e-3  5*1e-3      5*1e-3     ];
var_max     = [15*1e-3  13*1e-3 10*1e-3 15*1e-3  10e-3       10e-3      ];
var_default = zeros(1,length(var_names));
for i = 1:length(var_names)
     var_default(i) = syn.(var_names(i));
end


%% PRE-SETTING Style
style = struct();

[style.vScale,style.fScale,style.pScale] = deal(5e-3,1e-3,1e-3);
[style.vColor,style.fColor,style.pColor] = deal([0.6350 0.0780 0.1840],[0 0.4470 0.7410],[0.4940 0.1840 0.5560]);
[style.vVis  ,style.fVis  ,style.pVis  ] = deal(true,true,true);
style.aVis                               = true;

[style.plot_lim,style.setup_lim] = deal([[-40 150];[-40 120]]*1e-3,[[-25 25];[-25 120]]*1e-3);

%% PRE-SETTING evaluation positions
% pts = [[0;110] [34;104] [65;80] [70;20] [10;-20]] * 1e-3;
pts = [[0;90] [30;80] [50;60] [65;10] [20;-20]] * 1e-3;

%% Preliminary Calculations (run only once per session) (takes long)

% must remove [var_names] from the pre-calculation (need to be symbolic)
presyn = syn;
for i = 1:length(var_names)
    presyn = rmfield(presyn,var_names(i));
end

predyn_B = tendon_symbolic(presyn,1); %the "preliminary" dynamics (unfinished, lacks config data)
predyn_C = tendon_symbolic(presyn,2);

%% generate ui
ui = struct();

ui.fig = uifigure('Name', 'Finger Dynamics GUI','Position',[150 100 1600 900]);
ui.g_fig   = uigridlayout(ui.fig);
ui.g_fig.RowHeight = {'16x','2x','1x','1x'};
ui.g_fig.ColumnWidth = {'3x','2x','16x','3x'};
ui.g_fig.RowSpacing = 5;
ui.g_fing.ColumnSpacing = 5;

% setup axes (shows setup)
ui.ax_setup = uiaxes(ui.g_fig);
ui.ax_setup.Layout.Row = 1;
ui.ax_setup.Layout.Column = [1 2];
xlim(ui.ax_setup,style.setup_lim(1,:));
ylim(ui.ax_setup,style.setup_lim(2,:));
set(ui.ax_setup,'xticklabel',[])
set(ui.ax_setup,'yticklabel',[])
daspect(ui.ax_setup,[1 1 1])

% plot axes (shows flowers, etc)
ui.ax_plot = uiaxes(ui.g_fig);
ui.ax_plot.Layout.Row = [1 4];
ui.ax_plot.Layout.Column = 3;
% set(ui.ax_plot.Title,'String',"CONCEPT ["+syn.concept+"]");
% pbaspect(ui.ax_plot,[2 1 1])
xlim(ui.ax_plot,style.plot_lim(1,:));
ylim(ui.ax_plot,style.plot_lim(2,:));
set(ui.ax_plot,'xticklabel',[])
set(ui.ax_plot,'yticklabel',[])
set(ui.ax_plot,'xtick',style.plot_lim(1,1):0.005:style.plot_lim(1,2))
set(ui.ax_plot,'ytick',style.plot_lim(2,1):0.005:style.plot_lim(2,2))
grid(ui.ax_plot,'on');
daspect(ui.ax_plot,[1 1 1])

% ui: sliders (adjust syn variables)
ui.p_syn = uipanel(ui.g_fig,'Title','Synthesis Variables');
ui.p_syn.Scrollable = 'on';
ui.p_syn.Layout.Row = [2 4];
ui.p_syn.Layout.Column = 1;
ui.g_syn = uigridlayout(ui.p_syn);
ui.g_syn.RowHeight = {35,35,35,35,35,35};
ui.g_syn.ColumnWidth = {'1x','5x'};
ui.g_syn.Scrollable = 'on';
ui.g_syn.RowSpacing = 0;
ui.g_syn.ColumnSpacing = 0;

ui.syn = struct();
for i=1:length(var_names)
    ui.syn.(var_names(i)) = uislider(ui.g_syn,'Limits',[var_min(i) var_max(i)], ...
        'ValueChangingFcn',@(src,event)updatesyn(src,event,var_names(i)),'FontSize',10,'Value',var_default(i));
    ui.syn.(var_names(i)).MajorTicks = [var_min(i) var_max(i)];
    % ui.syn.var_names(i).MinorTicks = [];
    % ui.syn.var_names(i).MajorTickLabels = [];
    ui.syn.(var_names(i)).Layout.Row = i;
    ui.syn.(var_names(i)).Layout.Column = 2;

    ui.lbl = uilabel(ui.g_syn);
    ui.lbl.Text = var_names(i);
    ui.lbl.WordWrap = "on";
    ui.lbl.Layout.Row = i;
    ui.lbl.Layout.Column = 1;
end
% INPUTS
% ui: discrete variables
ui.p_discr = uipanel(ui.g_fig,'Title','Discrete Param.');
ui.p_discr.Scrollable = 'on';
ui.p_discr.Layout.Row = [2 3];
ui.p_discr.Layout.Column = 2;
ui.g_discr = uigridlayout(ui.p_discr);
ui.g_discr.RowHeight = {35,35,35,35,35,35};
ui.g_discr.ColumnWidth = {'2x','3x'};
ui.g_discr.Scrollable = 'on';
ui.g_discr.RowSpacing = 0;
ui.g_discr.ColumnSpacing = 0;

ui.discr = struct();

% concept selector (TODO make this better)
ui.discr.concept = uidropdown(ui.g_discr,"Items",["B+","C+"],"ValueChangedFcn",@(src,event)updatesyn(src,event,"concept"));
if(syn.concept == 1)
    ui.discr.concept.Value = "B+";
else
    ui.discr.concept.Value = "C+";
end
ui.discr.concept.Layout.Row = 1;
ui.discr.concept.Layout.Column = 2;
ui.discr.concept_lbl = uilabel(ui.g_discr);
ui.discr.concept_lbl.Text = "CONCEPT";
ui.discr.concept_lbl.Layout.Row = 1;
ui.discr.concept_lbl.Layout.Column = 1;

% SOLVE (button)
ui.solve = uibutton(ui.g_fig,"Text","Solve >>","ButtonPushedFcn",@updatesol);
ui.solve.Layout.Row = 4;
ui.solve.Layout.Column = 2;

% OUTPUTS
% ui: calculations panel
ui.p_calc = uipanel(ui.g_fig,'Title','Calculations');
ui.p_calc.Scrollable = 'on';
ui.p_calc.Layout.Row = [1 2];
ui.p_calc.Layout.Column = 4;
ui.g_calc = uigridlayout(ui.p_calc);
% ui.g_calc.RowHeight = {35,35,35,35,35,35};
ui.g_calc.ColumnWidth = {'1x'};
ui.g_calc.Scrollable = 'on';
ui.g_calc.RowSpacing = 5;
ui.g_calc.ColumnSpacing = 0;

ui.calc = [];
for i = 1:length(pts)
    ui.calc = [ui.calc uitextarea(ui.g_calc,"Value","Input Point #"+string(i))];
    ui.calc(i).Layout.Row = i;
    ui.calc(i).Layout.Column = 1;
    ui.calc(i).Editable = "off";
    ui.calc(i).FontSize = 10;
end

% ui: visibility control
ui.p_vis = uipanel(ui.g_fig,'Title','Visibility');
% ui.p_vis.Scrollable = 'on';
ui.p_vis.Layout.Row = 3;
ui.p_vis.Layout.Column = 4;
ui.g_vis = uigridlayout(ui.p_vis);
ui.g_vis.RowHeight = {15};
ui.g_vis.ColumnWidth = {'1x','1x','1x','1x'};
% ui.g_vis.Scrollable = 'on';
ui.g_vis.RowSpacing = 0;
ui.g_vis.ColumnSpacing = 0;
ui.g_vis.Padding = [3 3 3 3];

vis_names = ["vVis" "fVis" "pVis" "aVis"];
ui.vis = struct();
for i = 1:length(vis_names)
    ui.(vis_names(i)) = uicheckbox(ui.g_vis,'Text',vis_names(i),'ValueChangedFcn',@(src,event)updatestyle(src,event,vis_names(i)),'Value',true);
    ui.Layout.Row = 1;
    ui.Layout.Column = i;
end

% ui: save
ui.save = uibutton(ui.g_fig,'Text','Save',"ButtonPushedFcn",@(src,event) uisave);
drawnow;

show_setup(ui.ax_setup,syn,style);

%% Initial Population
show_setup(ui.ax_setup,syn,style);
updatesol(0,0);
% dyns = get_dyns(predyn_B,predyn_C,syn,pts);
% calcs = calc_flowers(dyns,syn);
% show_flowers(ui.ax_plot,calcs,style);
%% Member functions
write_calcs(ui,calcs,style);
% get full dynamics from input positions
function dyns = get_dyns(predyn_B,predyn_C,syn,pts)
    if abs(syn.concept) == 1
        disp("[GET-DYNS] Calculating dynamics for CONCEPT B...")
        predyn = predyn_B;
    else
        disp("[GET-DYNS] Calculating dynamics for CONCEPT B...")
        predyn = predyn_C;
    end
    
    dyns = [];
    for i = 1:length(pts(1,:))
        disp("[DYNS] Solving for point ["+strjoin(string(pts(:,i))) + "]");
        dyns = [dyns get_dyn(predyn,syn,pts(:,i))];
    end
    disp("[GET-DYNS] Done.")
end

% Calculate flower
function calcs = calc_flowers(dyns,syn)
    calcs = [];
    disp("[CALC] Rolling out calculations...")
    for i = 1:length(dyns)
        calcs = [calcs calc_flower(dyns(i),syn)];
        disp("[CALC] Point #"+string(i)+" done")
    end
end

% Show flowers
function show_flowers(ax,calcs,style)
    cla(ax);
    hold(ax,"on");
    for i = 1:length(calcs)
        show_flower(ax,calcs(i),style);
        disp("[SHOW-FLOWER] Drawing Flower #"+string(i));
        % disp(calcs(i).maxv_v)
    end
end

function write_calcs(ui,calcs,style)
    outputs(1:length(calcs)) = "";
    for i = 1:length(calcs)
        outputs(i) = outputs(i) + sprintf("[P%d] [%.0f,%.0f %.0fdeg] Q:[%.2f,%.2f](mm)\n",i,calcs(i).pos(1)*1000,calcs(i).pos(2)*1000,rad2deg(calcs(i).pos(3)),calcs(i).q(1)*1000,calcs(i).q(2)*1000);
        outputs(i) = outputs(i) + sprintf("J_fing = [%s;%s]",strjoin(string(calcs(i).J_fing(1,:))),strjoin(string(calcs(i).J_fing(2,:))));
        if calcs(i).singular;  outputs(i) = outputs(i) + "<SINGULAR>\n"; else; outputs(i) = outputs(i) + "\n"; end
    end
    % disp(outputs(2));
    for i = 1:length(calcs)
        % ui.calc(i).Value="POINT #"+string(i)+newline+jsonencode(calcs(i),"PrettyPrint",true);
        ui.calc(i).Value = outputs(i);
    end
    drawnow;
end
% Save Data
function save_data(ui,syn,calcs)
    nowstr = datestr(now,'yyyy-mm-dd_HH-MM-SS');
    dir_string = "./figures/"+nowstr;
    mkdir(dir_string);
    disp("[SAVE] Saving data to " + dir_string);
    exportgraphics(ui.ax_plot ,dir_string + "/results.png");
    exportgraphics(ui.ax_setup,dir_string + "/setup.png");

    f_params = fopen(dir_string+"/params.json",'wt');
    fprintf(f_params,jsonencode(syn, "PrettyPrint", true))
    
    f_calcs = fopen(dir_string+"/calculations.json",'wt');
    fprintf(f_calcs,jsonencode(calcs,"PrettyPrint",true));
end

%% callback function(s)
function updatesyn(~,event,name)
    if name == "null"
        % do nothing
    elseif name == "concept"
        if event.Value(1) == "B"
            evalin('base',"syn.concept = 1;");
        else
            evalin('base',"syn.concept = 2;");
        end

    else
        evalin('base',sprintf("syn.%s = %f;",name,event.Value));
        % disp(action);
    end
    evalin('base','show_setup(ui.ax_setup,syn,style)');
end

function updatesol(~,~)
    disp("[UI-SOLVE] SOLVE STARTED")
    evalin('base','dyns = get_dyns(predyn_B,predyn_C,syn,pts);');
    evalin('base','calcs = calc_flowers(dyns,syn);');
    evalin('base','show_flowers(ui.ax_plot,calcs,style);');
    evalin('base','write_calcs(ui,calcs);')
    disp("[UI-SOLVE] SOLVE FINISHED")
end

function updatestyle(~,event,name)
    disp("[VIS] "+name + " Changed to " + event.Value)
    evalin('base',sprintf("style.%s=%d;",name,event.Value));
    evalin('base','show_flowers(ui.ax_plot,calcs,style);');
    evalin('base','write_calcs(ui,calcs,style);');
end

function uisave(~,~)
    evalin('base','save_data(ui,syn,calcs);');
end
