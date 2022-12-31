function [q_time, velocity_t, theta_t, theta_d_t, time, pixels] = Compute_trajectory(q_start, q_end, Kh, L, map, limits, init_q, velocity_t, init_theta, theta_d_t, time, init_pixels)

    q_current = q_start;
    q_time = [init_q;q_start(1:2)'];
    theta_t = [init_theta;q_start(3)];
    pixels = [init_pixels;get_pixels_coords(q_start(1), q_start(2))];


    c = 0;
    d = 0;
    while vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4) > 160  % meters to stop
        error = sqrt((q_end(1)-q_current(1))^2 + (q_end(2)-q_current(2))^2);
        Kv = 15*10^(-2)/error; % The constraint will not be violated.       

        if vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4) < 200
            Kv = Kv/100;
        end

        [v, gamma] = Rover_commands(q_end, q_current, Kv, Kh); % The velocity is only feedback driven, thus it will be the maximum when it is at the begininning.
        velocity_t = [velocity_t; v];

        commands = [v; gamma];
        
        if v > 20*10^(-2)
            disp('aborted')
            break
        end
        [q_current, variation, int_time] = update_kinematic_model(q_current, commands, L);
        time = time + int_time;
        if c == 1000
            
            disp(vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)])),4)
            
            theta_t = [theta_t; q_current(3)];
            theta_d_t = [theta_d_t; variation(3)];
            q_time = [q_time; [q_current(1) q_current(2)]];
            pixels = [pixels; get_pixels_coords(round(q_current(1)), round(q_current(2)))];
            c = 0;
        end

        if d == 10000 || d == 0
            plot_trajectory(map, q_time, limits)
            d = 0;

        end
        c = c + 1;
        d = d + 1;
    end
end











