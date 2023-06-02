

function [path_planned, coords] = A_star(n_start_pixels, n_goal_pixels, mapp, limits, demMap, slope, limited_memory)                                          
    % INPUT: n_start: Starting node, n_goal: Goal node, ObstacleMap (0-255
    % are the obstacles values. Map Dimension : 10001x10001, limits: is the
    % vector with the map points for the plot of the visited nodes.
    % OUTPUT: Sequence of nodes in a matrix form, that contain the nodes to
    % visit in order to reach n_goal from n_start.

%   
    NN = 5;
    neigh_confs = [1 0;
                   0 1;
                   1 1;
                  -1 -1;
                  -1 0;
                  0 -1;
                  1 -1;
                  -1 1];
    path_planned = 0;
    
    threshold = 1/0;
    n_start = Node(n_start_pixels);
    n_goal = Node(n_goal_pixels);
    openSet = containers.Map; % Empty Map container to collect the nodes to analyze. Keys are the F values, and the Values are the Node object
                             % Empty matrices to contain all the path points.
    n_start.g = 0;

    n_start.h = n_start.get_heuristic_value(n_goal);
    
    if limited_memory == true
        threshold = n_start.h; % This threshold is needed to control the memory usage. 
    end

    n_start.f = n_start.g + n_start.h;

    openSet(pixelify(n_start.pixels)) = n_start; % Container has the size (n_elements x 1)
    expanded_nodes = containers.Map;
    expanded_nodes(pixelify(n_start.pixels)) = n_start;
   
    d = 0; %variable to define the visited nodes

    while size(openSet,1) ~= 0
        d = d + 1;
        n_curr = get_lowest_f_node(openSet);
        % check if goal

        if sum(n_curr.pixels == n_goal.pixels) == 2
            disp('Solution has been found........')

            path_planned = PathReconstruction(n_curr, n_start);
            break
        end

        % Remove ncurr from openSet
        pix_to_rem = pixelify(n_curr.pixels);
  
        openSet = remove(openSet, pix_to_rem);
        if d == 10000
           disp('OpenSet size: \n')
           disp(size(openSet,1))
        end
        if d == 200000
            
            coords = get_coords_from_container(expanded_nodes);
            plot_trajectory(mapp, coords, limits, 'yo')
            d = 0;
        end
        
        for conf_idx = 1:(size(neigh_confs, 1))
            curr_conf = neigh_confs(conf_idx, :)';
            new_pix = n_curr.pixels + curr_conf; % New set of candidate pixels.

            if slope == 0
                correct = check_candidate_validity(new_pix, mapp);
                additional_cost = 0;
            else
                [correct, additional_cost] = check_candidate_validity_with_slope(new_pix, n_curr.pixels, mapp, demMap, slope);     
            end
                
            key = pixelify(new_pix);

            if isKey(expanded_nodes, key)
                n_query = expanded_nodes(key);
            else
                n_query = Node(new_pix); % Initialize new node object
            end
            if correct == true
    
                if abs(sum(curr_conf)) == 2 || abs(sum(curr_conf)) == 0 % Which means that is requested a diagonal motion
                    cost = sqrt(2)*(NN + additional_cost);
                else
                    cost = NN + additional_cost;
                g_tentative = n_curr.g + cost;
                end
            else
                g_tentative = 1/0;
                expanded_nodes(key) = n_curr; % obstacle is expanded
            end

            if g_tentative <= n_query.g
                n_query.g = g_tentative;
                n_query.h = n_query.get_heuristic_value(n_goal);
                n_query.f = n_query.g + n_query.h;  % 10 is a weight that kill the heuristic admissability
                n_query.prev = n_curr; % This works as a pointer to another object.
                expanded_nodes(key) = n_curr;
                if isKey(openSet, key) == false && n_query.h < threshold% If the node is not in the list of the TO VISIT nodes, it must be added.
                    
                    openSet(key) = n_query;
                    
                end
            end
         
        end
    end

    coords = get_coords_from_container(expanded_nodes); % Expanded nodes
    plot_trajectory(mapp, coords, limits, 'yo')

end


function n_curr = get_lowest_f_node(container)
   % INPUT: openSet: Container Map, where the keys are f values and the
   % values are the node object.
   % OUTPUT: The current nbode with the lowest f value.
   value = cellfun(@getNum, values(container));
% 
   [~, index] = min(value);
    
   chiavi = keys(container);
    
   n_curr = container(chiavi{1, index});


end


function num = getNum(element)
   num = element.f;
end


function [k] = pixelify(pixels)
    % This function takes a pixel input, and establish a unique key to
    % detect the pixels (i_j) pixel format.

    k = string(pixels(1)) + '_' + string(pixels(2));
end

function correct = check_candidate_validity(conf, map)
    % This function tells if a neighbor could visited in principle, before
    % evaluating its proximity to the goal node.
    % INPUTs: conf: set of new pixel, map: Obstacles map.
    % OUTPUT: True if there are no issues with the evaluation, False
    % otherwise
    
    point = map(conf(1),conf(2)); 
    if point ~= 255 && point ~= 0 && conf(1) <= 10001 && conf(2) <= 10001 && conf(1) > 0 && conf(2) > 0  % Conf must be inside the map, and in that map point should not be white or black pixels.
        correct = true;
    else
        correct = false;
    end
    

end

function [correct, additional_cost] = check_candidate_validity_with_slope(conf, old_conf, map, demMap, alpha)
    % This function tells if a neighbor could visited in principle, before
    % evaluating its proximity to the goal node.
    % INPUTs: conf: set of new pixel, map: Obstacles map.
    % OUTPUT: True if there are no issues with the evaluation, False
    % otherwise

    
    if conf(1) <= 10001 && conf(2) <= 10001 && conf(1) > 0 && conf(2) > 0 && conf(2) > 3669 && conf(2) < 7469
        point = map(conf(1),conf(2)); 
        curr_conf = conf - old_conf;
        increment = demMap(conf(1),conf(2)) - demMap(old_conf(1), old_conf(2));
        
        if abs(sum(curr_conf)) == 2 || abs(sum(curr_conf)) == 0
            value = 5*sqrt(2);
            angle = atan2(increment, value*sqrt(2));
            
        else
            value = 5;
            angle = atan2(increment, value);
        end
        if point ~= 255 && point ~= 0 && angle <= alpha
            correct = true;
            additional_cost = sqrt(value^2 + increment^2) - value; % Longest side of the triangle is the additional cost.
        else
            correct = false;
            additional_cost = 0;
       
        end
    else
        correct = false;
        additional_cost = 0;
    
    end
end
    
   
        
       
   



function path = PathReconstruction(goal, start)
    % INPUT: goal: goal node, from which we want to back-iterate and
    % understand the path, start: Start node.
    % OUTPUT: path: Defined in cartesian coordinates.

    goal_m = get_cart_coords(goal.pixels(1), goal.pixels(2));
    path = [goal_m];

    current_node = goal;
    while current_node.pixels(1) ~= start.pixels(1) && current_node.pixels(2) ~= start.pixels(2)
 
        current_node = current_node.prev;
%         disp(current_node.pixels')
        current_node_m = get_cart_coords(current_node.pixels(1), current_node.pixels(2));
        path = [current_node_m;path];

    end


end


function coords = get_coords_from_container(container)
    % This function transform in a matrix form the pixels stored in a Map
    % data type.
   key_openSet = keys(container);   % Containers keys cells has the cell-like size (1 x n_elements)
   coords = zeros(size(container, 1), 2);
   for node_idx = 1:size(container,1)
       k = cell2mat(key_openSet(1, node_idx));
       pix = container(k).pixels';
       pix = get_cart_coords(pix(1), pix(2));
       coords(node_idx,:) = pix;
   end

    
end

