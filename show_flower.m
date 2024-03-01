function show_flower(ax,calc,style)
    hold(ax,"on");
    %% Actual Plotting!

    body_color = [0.4660 0.6740 0.1880]; %the "appearance" of the body changes with the calculations
    body_ls = "-";

    if(calc.singular)
        body_ls = "--";
    end
    if(calc.warn)
        body_color = "red";
    end

    % plot the manipulator as a set of lines (TODO: and circles)
    plot(ax,[calc.plot_points(1,1:end-1);calc.plot_points(1,2:end)],[calc.plot_points(2,1:end-1);calc.plot_points(2,2:end)],'Color',body_color,'LineStyle',body_ls,'LineWidth',2,'Marker','o')


    % flowers
    % scale into "~_points"
    if(~calc.singular)
        v_points = style.vScale*[calc.v_borders calc.v_borders(:,1)] + calc.plot_points(:,end);
        f_points = style.fScale   *[calc.f_borders calc.f_borders(:,1)] + calc.plot_points(:,end);
        p_points = style.pScale   *[calc.p_borders calc.p_borders(:,1)] + calc.plot_points(:,end);
        
        if(style.vVis)
            plot(ax,v_points(1,:),v_points(2,:),'Color',style.vColor)
        end
        if(style.fVis)
            plot(ax,f_points(1,:),f_points(2,:),'Color',style.fColor)
        end
        if(style.pVis)
            plot(ax,p_points(1,:),p_points(2,:),'Color',style.pColor)
        end
        % p_points
    end
    % annotation for max p/v/f stats
    if(style.aVis)
        % maxv annotation
        if(style.vVis)
            maxv_unit = [cos(calc.maxv.ang);sin(calc.maxv.ang)];
            maxv_points = [calc.plot_points(:,end) style.vScale*calc.maxv.v*maxv_unit + calc.plot_points(:,end) style.fScale*calc.maxv.f*maxv_unit + calc.plot_points(:,end) style.pScale*calc.maxv.p*maxv_unit + calc.plot_points(:,end)];
            if ((calc.maxv.ang == calc.maxp.ang) && style.pVis) || ((calc.maxv.ang == calc.maxf.ang) && style.fVis)
                text(ax,maxv_points(1,2),maxv_points(2,2),"Max V \rightarrow",'Color',style.vColor,'HorizontalAlignment','right',"FontWeight","bold");
            else
                plot(ax,maxv_points(1,:),maxv_points(2,:),'Color',style.vColor,'Marker','o');
                text(ax,maxv_points(1,2),maxv_points(2,2),"\leftarrow" + sprintf("v_{maxv}: %0.3f m/s" ,calc.maxv.v),'Color',style.vColor);
                text(ax,maxv_points(1,3),maxv_points(2,3),"\leftarrow" + sprintf("f_{maxv}: %0.3f N"   ,calc.maxv.f),'Color',style.fColor);
                text(ax,maxv_points(1,4),maxv_points(2,4),"\leftarrow" + sprintf("p_{maxv}: %0.3f Nm/s",calc.maxv.p),'Color',style.pColor);
            end
        end
    
        % maxf
        if(style.fVis)
            maxf_unit = [cos(calc.maxf.ang);sin(calc.maxf.ang)];
            maxf_points = [calc.plot_points(:,end) style.vScale*calc.maxf.v*maxf_unit + calc.plot_points(:,end) style.fScale*calc.maxf.f*maxf_unit + calc.plot_points(:,end) style.pScale*calc.maxf.p*maxf_unit + calc.plot_points(:,end)];
        
            if (calc.maxf.ang == calc.maxp.ang) && style.pVis
                text(ax,maxf_points(1,3),maxf_points(2,3),"Max F \rightarrow",'Color',style.fColor,'HorizontalAlignment','right',"FontWeight","bold");
            else
                plot(ax,maxf_points(1,:),maxf_points(2,:),'Color',style.fColor,'Marker','o');
                text(ax,maxf_points(1,2),maxf_points(2,2),"\leftarrow" + sprintf("v_{maxf}: %0.3f m/s" ,calc.maxf.v),'Color',style.vColor);
                text(ax,maxf_points(1,3),maxf_points(2,3),"\leftarrow" + sprintf("f_{maxf}: %0.3f N"   ,calc.maxf.f),'Color',style.fColor);
                text(ax,maxf_points(1,4),maxf_points(2,4),"\leftarrow" + sprintf("p_{maxf}: %0.3f Nm/s",calc.maxf.p),'Color',style.pColor);
            end    
        end
    
        % maxp
        if(style.pVis)
            maxp_unit = [cos(calc.maxp.ang);sin(calc.maxp.ang)];
            maxp_points = [calc.plot_points(:,end) style.vScale*calc.maxp.v*maxp_unit + calc.plot_points(:,end) style.fScale*calc.maxp.f*maxp_unit + calc.plot_points(:,end) style.pScale*calc.maxp.p*maxp_unit + calc.plot_points(:,end)];
            plot(ax,maxp_points(1,:),maxp_points(2,:),'Color',style.pColor,'Marker','o');
            text(ax,maxp_points(1,2),maxp_points(2,2),"\leftarrow" + sprintf("v_{maxp}: %0.3f m/s" ,calc.maxp.v),'Color',style.vColor);
            text(ax,maxp_points(1,3),maxp_points(2,3),"\leftarrow" + sprintf("f_{maxp}: %0.3f N"   ,calc.maxp.f),'Color',style.fColor);
            text(ax,maxp_points(1,4),maxp_points(2,4),"\leftarrow" + sprintf("p_{maxp}: %0.3f Nm/s",calc.maxp.p),'Color',style.pColor);
        end
        % add annotation for antagonistic force (and its percentage wrt max force)
    
        if(style.fVis)
            fant_points = [calc.plot_points(:,end) style.fScale*calc.fant_vec + calc.plot_points(:,end)];
            plot(ax,fant_points(1,:),fant_points(2,:),'black');
            text(ax,fant_points(1,2),fant_points(2,2),"\leftarrow" + sprintf("f_{ant}: %0.3f N"   ,norm(calc.fant_vec)),'Color','black')
        end    
    end


end