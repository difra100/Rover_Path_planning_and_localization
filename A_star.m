

function [path] = A_star(n_start_pixels, n_goal_pixels, obstacleMap)                                          
    % INPUT: n_start: Starting node, n_goal: Goal node, ObstacleMap (0-255
    % are the obstacles values. Map Dimension : 10001x10001.
    % OUTPUT: Sequence of nodes in a matrix form, that contain the nodes to
    % visit in order to reach n_goal from n_start.

% 
    NN = 1;
    neigh_confs = [1 0;
                   0 1;
                   1 1;
                  -1 -1;
                  -1 0;
                  0 -1;
                  1 -1;
                  -1 1];
    n_start = Node(n_start_pixels);
    n_goal = Node(n_goal_pixels);
    openSet = containers.Map % Empty Map container to collect the nodes to analyze. Keys are the F values, and the Values are the Node object
                             % Empty matrices to contain all the path points.
    n_start.g = 0;

    n_start.h = n_start.get_heuristic_value(n_goal);
    n_start.f = n_start.g + n_start.h;

    openSet(string(n_start.f)) = n_start; % Container has the size (n_elements x 1)

    while size(openSet,1) ~= 0
        n_curr = get_lowest_f_node(openSet);

        % check if goal
%         disp(sum(n_curr.pixels == n_goal.pixels))
        if sum(n_curr.pixels == n_goal.pixels) == 2
            disp('Solution has been found........')

            path = PathReconstruction(n_curr, n_start);
            break
        end

        % Remove ncurr from openSet
        openSet = remove(openSet, string(n_curr.f));

        for conf_idx = 1:(size(neigh_confs, 1))
            curr_conf = neigh_confs(conf_idx, :)';
            new_pix = n_curr.pixels + curr_conf; % New set of candidate pixels.
            correct = check_candidate_validity(new_pix, obstacleMap);

            if correct == true % Visitable nodes only
                n_query = Node(new_pix); % Initialize new node object

                if abs(sum(curr_conf)) == 2 || abs(sum(curr_conf)) == 0 % Which means that is requested a diagonal motion
                    cost = sqrt(2)*NN;
                else
                    cost = NN;
                g_tentative = n_curr.g + cost;
                end
                if g_tentative < n_query.g
                    n_query.g = g_tentative;
                    n_query.h = n_query.get_heuristic_value(n_goal);
                    n_query.f = n_query.g + n_query.h;
                    n_query.prev = n_curr; % This works as a pointer to another object.

                    in_set = check_if_node_in_list(n_query, openSet);
                    if in_set == false  % If the node is not in the list of the TO VISIT nodes, it must be added.
                        key = string(n_query.f);
                        openSet(key) = n_query;
                    end
                end
            end
         


        end

    end

    

end


function n_curr = get_lowest_f_node(openSet)
   % INPUT: openSet: Container Map, where the keys are f values and the
   % values are the node object.
   % OUTPUT: The current nbode with the lowest f value.
   minimum = 1/0;
   key_openSet = keys(openSet);   % Containers keys cells has the cell-like size (1 x n_elements)
   for node_idx = 1:size(openSet,1)
   
       if minimum > str2num(cell2mat(key_openSet(1, node_idx)))
           minimum =  str2num(cell2mat(key_openSet(1, node_idx)));
           n_curr = openSet(cell2mat(key_openSet(1, node_idx)));
       end

    
   end


end


function in_set = check_if_node_in_list(query_n, openSet)
   % INPUT: openSet: Container Map, where the keys are f values and the
   % values are the node object, query_n:This is the node we want to check,
   % OUTPUT: The current nbode with the lowest f value.
   in_set = false;
   key_openSet = keys(openSet);   % Containers keys cells has the cell-like size (1 x n_elements)
   for node_idx = 1:size(openSet,1)
   
      n_current = openSet(cell2mat(key_openSet(1, node_idx)));
      
      if sum(n_current.pixels == query_n.pixels) == 2
        in_set = true;
        break
        
      end
      

    
   end


end

function correct = check_candidate_validity(conf, map)
    % This function tells if a neighbor could visited in principle, before
    % evaluating its proximity to the goal node.
    % INPUTs: conf: set of new pixel, map: Obstacles map.
    % OUTPUT: True if there are no issues with the evaluation, False
    % otherwise
    point = map(conf(1),conf(2)); 
    if point ~= 255 && point ~= 0 && conf(1) <= 10001 && conf(2) <= conf(2) <= 10001 && conf(1) > 0 && conf(2) > 0  % Conf must be inside the map, and in that map point should not be white or black pixels.
        correct = true;
    else
        correct = false;
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




















% 
% 
% 
