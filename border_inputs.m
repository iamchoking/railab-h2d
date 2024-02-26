function border = border_inputs(max1,min1,max2,min2,points)
%BORDER_INPUT Summary of this function goes here
%   Detailed explanation goes here
% marker_1 = [max1;max2];
% marker_2 = [min1;max2];
% marker_3 = [min1;min2];
% marker_4 = [max1;min2];

unit_size = round(points/4);

down_1 = linspace(max1,min1,unit_size+1);
up_1   = linspace(min1,max1,unit_size+1);
down_2 = linspace(max2,min2,unit_size+1);
up_2   = linspace(min2,max2,unit_size+1);

border = [down_1(1:end-1)        min1*ones(1,unit_size) up_1(1:end-1)          max1*ones(1,unit_size);
          max2*ones(1,unit_size) down_2(1:end-1)        min2*ones(1,unit_size) up_2(1:end-1)];

end

