
%% Designating variables (must be a initialized in variable "syn")
var_names   = ["r1m"    "r1p"   "r1d"   "r2m"   "taua1_max" "taua2_max" "rym" "ryp" "Tyi" "ky"  "rxd"];
var_min     = [5*1e-3   5*1e-3  5*1e-3  5*1e-3  5*1e-3      5*1e-3      5e-3   5e-3  0.1   18   5e-3];
var_max     = [15*1e-3  13*1e-3 10*1e-3 15*1e-3  10e-3       10e-3      15e-3  13e-3 50    4500 15e-3];
var_default = zeros(1,length(var_names));
for i = 1:length(var_names)
     var_default(i) = syn.(var_names(i));
end

%% generate ui
ui = struct();

ui.concept_struct = struct();
ui.concept_struct.("B_op")   =  1;
ui.concept_struct.("C_op")   =  2;
ui.concept_struct.("B_cl")   = -1;
ui.concept_struct.("C_cl")   = -2;

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
ui.g_syn.RowHeight = {35,35,35,35,35,35,35,35,35,35,35,35};
ui.g_syn.ColumnWidth = {'1x','5x'};
ui.g_syn.Scrollable = 'on';
ui.g_syn.RowSpacing = 0;
ui.g_syn.ColumnSpacing = 0;

ui.syn = struct();
for i=1:length(var_names)
    ui.syn.(var_names(i)) = uislider(ui.g_syn,'Limits',[var_min(i) var_max(i)], ...
        'ValueChangedFcn',@(src,event)updatesyn(src,event,var_names(i)),'FontSize',10,'Value',syn.(var_names(i)));
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
ui.p_discr.Layout.Row = 2;
ui.p_discr.Layout.Column = 2;
ui.g_discr = uigridlayout(ui.p_discr);
ui.g_discr.RowHeight = {35,35,35,35,35,35};
ui.g_discr.ColumnWidth = {'2x','3x'};
ui.g_discr.Scrollable = 'on';
ui.g_discr.RowSpacing = 0;
ui.g_discr.ColumnSpacing = 0;

ui.discr = struct();

% concept selector (TODO make this better)
ui.discr.concept = uidropdown(ui.g_discr,"Items",string(fieldnames(ui.concept_struct)),"ValueChangedFcn",@(src,event)updatesyn(src,event,"concept"));
ui.discr.concept.Value = "B_op";
% if(syn.concept == 1)
%     ui.discr.concept.Value = "B+";
% else
%     ui.discr.concept.Value = "C+";
% end
ui.discr.concept.Layout.Row = 1;
ui.discr.concept.Layout.Column = 2;
ui.discr.concept_lbl = uilabel(ui.g_discr);
ui.discr.concept_lbl.Text = "CONCEPT";
ui.discr.concept_lbl.Layout.Row = 1;
ui.discr.concept_lbl.Layout.Column = 1;

% REFRESH UI (button)
ui.solve = uibutton(ui.g_fig,"Text","Refresh","ButtonPushedFcn",@uirefresh);
ui.solve.Layout.Row = 3;
ui.solve.Layout.Column = 2;

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
% calcs = calc_flowers(dyns,syn);
% show_flowers(ui.ax_plot,calcs,style);
%% Member functions
% write_calcs(ui,calcs,style);
% save_data(ui,syn,calcs,style,pts,input_mode);
% tendon_dyn_numeric(syn,pts(1,:),input_mode);
% get full dynamics from input positions
refresh_ui(ui,syn,pts,style);

function s = concept_name(concept_struct,num)
    s = "NULL";
    % disp(concept_struct)
    names = string(fieldnames(concept_struct));
    for i = 1:length(names)
        if concept_struct.(names(i)) == num
            s = names(i);
        end
    end
end

function dyns = get_dyns(syn,pts,input_mode)
    dyns = [];
    for i = 1:length(pts(1,:))
        disp("[DYNS] Solving for configuration ["+strjoin(string(pts(:,i))) + "]");
        % dyns = [dyns get_dyn(predyn,syn,pts(:,i),input_mode)];
        dyns = [dyns tendon_dyn_numeric(syn,pts(:,i),input_mode)];
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

function outputs = write_calcs(ui,calcs,style,do_write)
    if nargin < 4
        do_write = true;
    end
    outputs(1:length(calcs)) = "";
    for i = 1:length(calcs)
        outputs(i) = outputs(i) + sprintf("[P%d] [%.0f,%.0f %.0fdeg] Q:[%.2f,%.2f](mm)\n",i,calcs(i).pos(1)*1000,calcs(i).pos(2)*1000,rad2deg(calcs(i).pos(3)),calcs(i).q(1)*1000,calcs(i).q(2)*1000);
        outputs(i) = outputs(i) + sprintf("J_fing = [%s;%s]",strjoin(string(calcs(i).J_fing(1,:))),strjoin(string(calcs(i).J_fing(2,:))));
        if calcs(i).singular;  outputs(i) = outputs(i) + "<SINGULAR>"+newline; else; outputs(i) = outputs(i) + newline; end

        % always write zero characteristics
        dat = calcs(i).zero;
        outputs(i) = outputs(i) + newline;
        outputs(i) = outputs(i) + sprintf("<zero>\n");
        outputs(i) = outputs(i) + sprintf("  tauq: [%.2f %.2f] N\n",dat.tauq(1),dat.tauq(2));
        outputs(i) = outputs(i) + sprintf("        (%.2fp %.2fp) max T\n",dat.tauq(1)/calcs(i).T_max(1)*100,dat.tauq(2)/calcs(i).T_max(2)*100);
        outputs(i) = outputs(i) + sprintf("  taux: %.2f N\n",dat.taux);
        outputs(i) = outputs(i) + sprintf("  dely: %.2f mm\n",dat.dely*1000);

        cat = ["v","f","p"];
        for j = 1:length(cat)
            if style.(cat(j)+"Vis")
                dat = calcs(i).("max"+cat(j));
                outputs(i) = outputs(i) + newline;
                outputs(i) = outputs(i) + sprintf("<max%s> %.1f deg\n",cat(j),rad2deg(dat.ang));
                outputs(i) = outputs(i) + sprintf("  v: %.2f m/s\n",dat.v);
                outputs(i) = outputs(i) + sprintf("  f: %.2f N\n",dat.f);
                outputs(i) = outputs(i) + sprintf("  p: %.2f W\n",dat.p);
                outputs(i) = outputs(i) + sprintf("  tauq: [%.2f %.2f] N\n",dat.tauq(1),dat.tauq(2));
                outputs(i) = outputs(i) + sprintf("  taux: %.2f N\n",dat.taux);
            end
        end
    end
    % disp(outputs(2));
    if do_write
        for i = 1:length(calcs)
            % ui.calc(i).Value="POINT #"+string(i)+newline+jsonencode(calcs(i),"PrettyPrint",true);
            ui.calc(i).Value = outputs(i);
        end
    end
    drawnow;
end
% Save Data
function save_data(ui,syn,calcs,style,pts,input_mode)
    nowstr = datestr(now,'yyyy-mm-dd_HH-MM-SS');
    dir_string = "./figures/"+concept_name(ui.concept_struct,syn.concept)+"/"+nowstr;
    mkdir(dir_string);
    disp("[SAVE] Saving data to " + dir_string);
    % photos
    exportgraphics(ui.ax_plot ,dir_string + "/results.png");
    exportgraphics(ui.ax_setup,dir_string + "/setup.png");

    % variables
    save(dir_string+"/syn","syn","style","pts","input_mode");

    % files
    f_params = fopen(dir_string+"/params.json",'wt');
    fprintf(f_params,jsonencode(syn, "PrettyPrint", true));
    
    f_calcs = fopen(dir_string+"/calcs_raw.json",'wt');
    fprintf(f_calcs,jsonencode(calcs,"PrettyPrint",true));

    f_readouts = fopen(dir_string+"/calcs_readouts.json",'wt');
    readouts= struct();
    for i = 1:length(ui.calc)
        readouts.("P"+string(i)) = sprintf("%s\n",strjoin(ui.calc(i).Value));
    end
    % disp(readouts);
    fprintf(f_readouts,jsonencode(readouts, "PrettyPrint", true));

    fclose(f_params);
    fclose(f_calcs);
    fclose(f_readouts);
end

function refresh_ui(ui,syn,pts,style)
    % INPUT UI
    % sync slider values to syn
    ui_syn_names = string(fieldnames(ui.syn));
    for i = 1:length(ui_syn_names)
        ui.syn.(ui_syn_names(i)).Value = syn.(ui_syn_names(i));
    end

    % sync concept dropdown to syn
    ui.discr.concept.Value = concept_name(ui.concept_struct,syn.concept);
    % ui_concept_names = string(fieldnames(ui.concept_struct));
    % for i = 1:length(ui_concept_names)
    %     if ui.concept_struct.(ui_concept_names(i)) == syn.concept
    %         ui.discr.concept.Value = ui_concept_names(i);
    %     end
    % end

    % show the setup again
    show_setup(ui.ax_setup,syn,style);

    % OUTPUT UI (TODO)
    % sync number of input points and number of panels (TODO)    

    % sync visibility checkboxes to style(TODO)
   
    % draw the flower / write calcs again
    % evalin('base','show_flowers(ui.ax_plot,calcs,style);');

    disp("[UI-REFRESH] Refresehed UI to current syn")
end

%% callback function(s)
function updatesyn(~,event,name)
    if name == "null"
        % do nothing
    elseif name == "concept"
        evalin('base',sprintf("syn.concept = ui.concept_struct.%s;",event.Value));
        % if event.Value(1) == "B"
        %     evalin('base',"syn.concept = 1;");
        % else
        %     evalin('base',"syn.concept = 2;");
        % end
    else
        evalin('base',sprintf("syn.%s = %f;",name,event.Value));
        % disp(action);
        if name == "rxd"
            evalin('base',sprintf("syn.%s = %f;","rxp",event.Value*(9/11)));
        end
    end
    evalin('base','show_setup(ui.ax_setup,syn,style)');
end

function updatesol(~,~)
    disp("[UI-SOLVE] SOLVE STARTED")
    evalin('base','dyns = get_dyns(syn,pts,input_mode);');
    evalin('base','calcs = calc_flowers(dyns,syn);');
    evalin('base','show_flowers(ui.ax_plot,calcs,style);');
    evalin('base','write_calcs(ui,calcs,style);');
    disp("[UI-SOLVE] SOLVE FINISHED");
end

function updatestyle(~,event,name)
    disp("[VIS] "+name + " Changed to " + event.Value)
    evalin('base',sprintf("style.%s=%d;",name,event.Value));
    evalin('base','show_flowers(ui.ax_plot,calcs,style);');
    evalin('base','write_calcs(ui,calcs,style);');
end

function uisave(~,~)
    evalin('base','save_data(ui,syn,calcs,style,pts,input_mode);');
end

function uirefresh(~,~)
    evalin('base','refresh_ui(ui,syn,pts,style)')
end