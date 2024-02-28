function show_flower(ax,calc,style)
    hold(ax,"on");
    %% Actual Plotting!

    % plot the manipulator as a set of lines (TODO: and circles)
    plot(ax,[calc.plot_points(1,1:end-1);calc.plot_points(1,2:end)],[calc.plot_points(2,1:end-1);calc.plot_points(2,2:end)],"cyan",'LineWidth',3)

    % flowers
    % scale into "~_points"
    
    v_points = style.vScale*[calc.v_borders calc.v_borders(:,1)] + calc.plot_points(:,end);
    f_points = style.fScale   *[calc.f_borders calc.f_borders(:,1)] + calc.plot_points(:,end);
    p_points = style.pScale   *[calc.p_borders calc.p_borders(:,1)] + calc.plot_points(:,end);
    
    plot(ax,v_points(1,:),v_points(2,:),'Color',style.vColor)
    plot(ax,f_points(1,:),f_points(2,:),'Color',style.fColor)
    plot(ax,p_points(1,:),p_points(2,:),'Color',style.pColor)
    % p_points
    
    % annotation for max p/v/f stats
    % maxv
    maxv_unit = [cos(calc.maxv_ang);sin(calc.maxv_ang)];
    maxv_points = [calc.plot_points(:,end) style.vScale*calc.maxv_v*maxv_unit + calc.plot_points(:,end) style.fScale*calc.maxv_f*maxv_unit + calc.plot_points(:,end) style.pScale*calc.maxv_p*maxv_unit + calc.plot_points(:,end)];
    plot(ax,maxv_points(1,:),maxv_points(2,:),'Color',style.vColor);
    scatter(ax,maxv_points(1,2:end),maxv_points(2,2:end),'Color',style.vColor);
    text(ax,maxv_points(1,2),maxv_points(2,2),"\leftarrow" + sprintf("v_{maxv}: %0.3f m/s" ,calc.maxv_v),'Color',style.vColor);
    % text(ax,maxv_points(1,3),maxv_points(2,3),"\leftarrow" + sprintf("f_{maxv}: %0.3f N"   ,calc.maxv_f),'Color',style.fColor);
    % text(ax,maxv_points(1,4),maxv_points(2,4),"\leftarrow" + sprintf("p_{maxv}: %0.3f Nm/s",calc.maxv_p),'Color',style.pColor);

    % maxf
    maxf_unit = [cos(calc.maxf_ang);sin(calc.maxf_ang)];
    maxf_points = [calc.plot_points(:,end) style.vScale*calc.maxf_v*maxf_unit + calc.plot_points(:,end) style.fScale*calc.maxf_f*maxf_unit + calc.plot_points(:,end) style.pScale*calc.maxf_p*maxf_unit + calc.plot_points(:,end)];
    plot(ax,maxf_points(1,:),maxf_points(2,:),'Color',style.fColor);
    scatter(ax,maxf_points(1,2:end),maxf_points(2,2:end),'Color',style.fColor);
    % text(ax,maxf_points(1,2),maxf_points(2,2),"\leftarrow" + sprintf("v_{maxf}: %0.3f m/s" ,calc.maxf_v),'Color',style.vColor);
    text(ax,maxf_points(1,3),maxf_points(2,3),"\leftarrow" + sprintf("f_{maxf}: %0.3f N"   ,calc.maxf_f),'Color',style.fColor);
    % text(ax,maxf_points(1,4),maxf_points(2,4),"\leftarrow" + sprintf("p_{maxf}: %0.3f Nm/s",calc.maxf_p),'Color',style.pColor);


    % maxp
    maxp_unit = [cos(calc.maxp_ang);sin(calc.maxp_ang)];
    maxp_points = [calc.plot_points(:,end) style.vScale*calc.maxp_v*maxp_unit + calc.plot_points(:,end) style.fScale*calc.maxp_f*maxp_unit + calc.plot_points(:,end) style.pScale*calc.maxp_p*maxp_unit + calc.plot_points(:,end)];

    if(calc.maxp_ang == calc.maxv_ang)
        text(ax,maxp_points(1,3),maxp_points(2,3),"\leftarrow" + sprintf("f_{maxp}: %0.3f N"   ,calc.maxp_f),'Color',style.fColor);
    elseif(calc.maxp_ang == calc.maxf_ang)
        text(ax,maxp_points(1,2),maxp_points(2,2),"\leftarrow" + sprintf("v_{maxp}: %0.3f m/s" ,calc.maxp_v),'Color',style.vColor);
    else
        plot(ax,maxp_points(1,:),maxp_points(2,:),'Color',style.pColor);
        scatter(ax,maxp_points(1,2:end),maxp_points(2,2:end),'Color',style.pColor);
        text(ax,maxp_points(1,2),maxp_points(2,2),"\leftarrow" + sprintf("v_{maxp}: %0.3f m/s" ,calc.maxp_v),'Color',style.vColor);
        text(ax,maxp_points(1,3),maxp_points(2,3),"\leftarrow" + sprintf("f_{maxp}: %0.3f N"   ,calc.maxp_f),'Color',style.fColor);
    end

    text(ax,maxp_points(1,4),maxp_points(2,4),"\leftarrow" + sprintf("p_{maxp}: %0.3f Nm/s",calc.maxp_p),'Color',style.pColor);
    text(ax,maxp_points(1,4),maxp_points(2,4),"Max P \rightarrow",'Color',style.pColor,'HorizontalAlignment','right',"FontWeight","bold");


    % add annotation for antagonistic force (and its percentage wrt max force)
    fant_points = [calc.plot_points(:,end) style.fScale*calc.fant_vec + calc.plot_points(:,end)];
    plot(ax,fant_points(1,:),fant_points(2,:),'black');
    text(ax,fant_points(1,2),fant_points(2,2),"\leftarrow" + sprintf("f_{ant}: %0.3f N"   ,norm(calc.fant_vec)),'Color','black')
    
end