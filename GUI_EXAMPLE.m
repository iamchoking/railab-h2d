function sliderApp
fig = uifigure("Position",[100 100 300 250]);
g = uigridlayout(fig);
g.RowHeight = {'1x','fit'};
g.ColumnWidth = {'1x','fit','1x'};

cg = uigauge(g);
cg.Layout.Row = 1;
cg.Layout.Column = [1 3];
%%
sld = uislider(g, ...
    "ValueChangedFcn",@(src,event)updateGauge(src,event,cg));
sld.Layout.Row = 2;
sld.Layout.Column = 2;
end

function updateGauge(src,event,cg)
cg.Value = event.Value;
end