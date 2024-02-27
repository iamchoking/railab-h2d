function show_setup(ax_setup,syn,style)
    cla(ax_setup);
    hold(ax_setup,"on");
    
    % pbaspect(ax_setup,[1 1 1])
    
    % string 1: blue / string 2: red / coupler & antagonistic: black
    m_center = [0;0];
    p_center = [0;syn.l1];
    d_center = [0;syn.l1+syn.l2];
    e_center = [0;syn.l1+syn.l2+syn.l3];
    
    % [FINGER KINEMATICS] (black)
    finger_points = [zeros(1,4);0 syn.l1 syn.l1+syn.l2 syn.l1+syn.l2+syn.l3];
    plot   (ax_setup,finger_points(1,:),finger_points(2,:),'black','LineWidth',3,'Marker','o')
    text(ax_setup,0,syn.l1/2,'  l1: '+string(syn.l1*1e3) + 'mm','Color','black');
    text(ax_setup,0,syn.l1 + syn.l2/2,'  l2: '+string(syn.l2*1e3) + 'mm','Color','black');
    text(ax_setup,0,syn.l1 + syn.l2 + syn.l3/2,'  l3: '+string(syn.l3*1e3) + 'mm','Color','black');
    
    % [STRING 1] (blue)
    plot_circle(ax_setup,m_center,syn.r1m,'blue',':');
    text(ax_setup,syn.r1m,0,'\leftarrow r1m: '+string(syn.r1m*1e3) + 'mm','Color','blue');
    plot_circle(ax_setup,p_center,syn.r1p,'blue',':');
    text(ax_setup,syn.r1p,p_center(2),'\leftarrow r1p: '+string(syn.r1p*1e3) + 'mm','Color','blue');
    plot_circle(ax_setup,d_center,syn.r1d,'blue',':');
    text(ax_setup,syn.r1d,d_center(2),'\leftarrow r1d: '+string(syn.r1d*1e3) + 'mm','Color','blue');
    
    if(abs(syn.concept) == 1)
        % ground to r1m (concept B)
        plot(ax_setup,[syn.r1m syn.r1m],[0 style.setup_lim(2,1)],'Color','blue','Marker','o');
        
        % arc in r1m (concept B)
        beta_r1mp = asin((syn.r1m-syn.r1p)/syn.l1);
        plot_circle(ax_setup,m_center,syn.r1m,'blue','-',[-2*pi beta_r1mp]);
        
        % r1m to r1p (concept B)
        start_r1mp = m_center+syn.r1m * [cos(beta_r1mp);sin(beta_r1mp)];
        end_r1mp   = p_center+syn.r1p * [cos(beta_r1mp);sin(beta_r1mp)];
        plot(ax_setup,[start_r1mp(1) end_r1mp(1)],[start_r1mp(2) end_r1mp(2)],'Color','blue','Marker','o');
        % arc in r1p (concept B)
        beta_r1pd = asin((syn.r1p-syn.r1d)/syn.l2);
        plot_circle(ax_setup,p_center,syn.r1p,'blue','-',[-2*pi+beta_r1mp beta_r1pd]);
    else
        % ground to r1m (concept C)
        plot(ax_setup,[-syn.r1m -syn.r1m],[0 style.setup_lim(2,1)],'Color','blue','Marker','o');
        
        % arc in r1m (concept C)
        beta_r1mp = asin((syn.r1m+syn.r1p)/syn.l1);
        plot_circle(ax_setup,m_center,syn.r1m,'blue','-',[pi pi-beta_r1mp]);
        
        % r1m to r1p (concept C)
        start_r1mp = m_center+syn.r1m * [cos(pi-beta_r1mp);sin(pi-beta_r1mp)];
        end_r1mp   = p_center+syn.r1p * [cos(-beta_r1mp);sin(-beta_r1mp)];
        plot(ax_setup,[start_r1mp(1) end_r1mp(1)],[start_r1mp(2) end_r1mp(2)],'Color','blue','Marker','o');
        % arc in r1p (C)
        beta_r1pd = asin((syn.r1p-syn.r1d)/syn.l2);
        plot_circle(ax_setup,p_center,syn.r1p,'blue','-',[-2*pi-beta_r1mp beta_r1pd]);
    end
    
    % r1p to r1d (both concepts)
    start_r1pd = p_center+syn.r1p * [cos(beta_r1pd);sin(beta_r1pd)];
    end_r1pd   = d_center+syn.r1d * [cos(beta_r1pd);sin(beta_r1pd)];
    plot(ax_setup,[start_r1pd(1) end_r1pd(1)],[start_r1pd(2) end_r1pd(2)],'Color','blue','Marker','o');
    
    % arc in r1d (both)
    plot_circle(ax_setup,d_center,syn.r1d,'blue','-',[beta_r1pd pi/2]);
    
    % fixture
    scatter(ax_setup,[0],[d_center(2)+syn.r1d],100,'blue','Marker','X','LineWidth',2)
    
    % [STRING 2] (red)
    plot_circle(ax_setup,m_center,syn.r2m,'red',':');
    text(ax_setup,-syn.r2m,0,'r2m: '+string(syn.r2m*1e3) + 'mm \rightarrow','Color','red','HorizontalAlignment','right');
    
    % ground to r1m
    plot(ax_setup,[syn.r2m syn.r2m],[0 style.setup_lim(2,1)],'Color','red','Marker','o');
    % r1m arc
    plot_circle(ax_setup,m_center,syn.r2m,'red','-',[0,pi/2]);
    % fixture
    scatter(ax_setup,[0],[syn.r2m],100,'red','Marker','X','LineWidth',2)
    
    % [STRING X] (black)
    plot_circle(ax_setup,p_center,syn.rxp,'black',':');
    text(ax_setup,-syn.rxp,p_center(2),'rxp: '+string(syn.rxp*1e3) + 'mm \rightarrow','Color','black','HorizontalAlignment','right');
    plot_circle(ax_setup,d_center,syn.rxd,'black',':');
    text(ax_setup,-syn.rxd,d_center(2),'rxd: '+string(syn.rxd*1e3) + 'mm \rightarrow','Color','black','HorizontalAlignment','right');
    
    % fixture
    scatter(ax_setup,[0],[p_center(2)-syn.rxp],100,'black','Marker','X','LineWidth',2)
    % arc on rxp
    beta_rxpd = asin((syn.rxp+syn.rxd)/syn.l2);
    plot_circle(ax_setup,p_center,syn.rxp,'black','-',[-pi/2 beta_rxpd]);
    % rxp to rxd
    start_rxpd = p_center+syn.rxp * [cos(beta_rxpd);sin(beta_rxpd)];
    end_rxpd   = d_center+syn.rxd * [cos(beta_rxpd+pi);sin(beta_rxpd+pi)];
    plot(ax_setup,[start_rxpd(1) end_rxpd(1)],[start_rxpd(2) end_rxpd(2)],'Color','black','Marker','o');
    % arc on rxp
    plot_circle(ax_setup,d_center,syn.rxd,'black','-',[pi/2 beta_rxpd+pi]);
    
    % fixture
    scatter(ax_setup,0,d_center(2)+syn.rxd,100,'black','Marker','X','LineWidth',2)
    
    % string y (gray)
    color_gray = [0.5 0.5 0.5];
    plot_circle(ax_setup,p_center,syn.ryp,color_gray,':');
    text(ax_setup,-syn.ryp*cos(pi/4),p_center(2)+syn.ryp*sin(pi/4),'ryp: '+string(syn.ryp*1e3) + 'mm \rightarrow','Color',[0.5 0.5 0.5],'HorizontalAlignment','right');
    if(abs(syn.concept) == 1) % B
        plot_circle(ax_setup,m_center,syn.rym,color_gray,':');
        text(ax_setup,-syn.rym*cos(pi/4),syn.rym*sin(pi/4),'rym: '+string(syn.rym*1e3) + 'mm \rightarrow','Color',[0.5 0.5 0.5],'HorizontalAlignment','right');
        %TODO draw tangent lines
    end
    
    if(abs(syn.concept) == 1) % B
        % ground to rym (B only)
        plot(ax_setup,[-syn.rym -syn.rym],[0 style.setup_lim(2,1)],'Color',color_gray,'Marker','o');
        % arc on rym
        beta_rymp = asin((syn.rym-syn.ryp)/syn.l1);
        plot_circle(ax_setup,m_center,syn.rym,color_gray,'-',[pi-beta_rymp pi],10)
        % rym to ryp
        start_rymp = m_center+syn.rym * [cos(pi-beta_rymp);sin(pi-beta_rymp)];
        end_rymp   = p_center+syn.ryp * [cos(pi-beta_rymp);sin(pi-beta_rymp)];
        plot(ax_setup,[start_rymp(1) end_rymp(1)],[start_rymp(2) end_rymp(2)],'Color',color_gray,'Marker','o');
        % arc on ryp
        plot_circle(ax_setup,p_center,syn.ryp,color_gray,'-',[pi-beta_rymp pi/2])
    else
        % fixture
        scatter(ax_setup,0,p_center(2)-syn.ryp,100,color_gray,'Marker','X','LineWidth',2)
        % arc on ryp
        plot_circle(ax_setup,p_center,syn.ryp,color_gray,'-',[pi/2 3*pi/2])
    end
    
    %fixture
    scatter(ax_setup,0,p_center(2)+syn.ryp,100,color_gray,'Marker','X','LineWidth',2)

end

