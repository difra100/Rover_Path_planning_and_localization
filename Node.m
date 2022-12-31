classdef Node
    properties
        pixels; 
        f;
        g;
        h;
        prev;
    end
    methods 
        function obj = Node(pixels)
            if nargin > 0
             obj.pixels = pixels;
             obj.g = 1/0;
             obj.f = 1/0; 
            end
        end

        function h = get_heuristic_value(obj, goal) % Octile distance, admissable heuristic
            dx = abs(obj.pixels(1)-goal.pixels(1));
            dy = abs(obj.pixels(2)-goal.pixels(2));

            h = max(dx, dy) + (sqrt(2)-1)*min(dx,dy);
        end
    end
end
