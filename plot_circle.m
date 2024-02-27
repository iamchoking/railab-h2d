function plot_circle(ax,center,radius,color,appearance,angle,numPoints)
    if nargin < 4
        color = 'black';
    end

    if nargin < 5
        appearance = '-';
    end
    
    if nargin < 6
        angle = [0 2*pi];
    end

    if nargin < 7
        numPoints = 50;
    end

    th = linspace(angle(1),angle(2),numPoints);
    points = [center(1) + radius*cos(th);center(2) + radius*sin(th)];
    plot(ax,points(1,:),points(2,:),'Color',color,'LineStyle',appearance);
end

