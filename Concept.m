classdef Concept
    enumeration
        Red (255, 0, 0), Green (0, 255, 0), Blue (0, 0, 255), Black (0, 0, 0), White (255, 255, 255)
    end
    
    properties
        RGB
    end
    
    methods
        function obj = Color(rgb)
            obj.RGB = rgb;
        end
    end
end