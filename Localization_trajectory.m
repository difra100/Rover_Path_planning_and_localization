function [q_time, velocity_t, theta_t, theta_d_t, time, pixels, P_current] = Localization_trajectory(q_start, q_end, P_init, odometric_noise, Kh, L, map, limits, init_q, velocity_t, init_theta, theta_d_t, time, init_pixels, xLM, yLM, instr_var_noise, instr_noise, maxlen, kalman)
    % This function belongs to the task 3 part of the homework. It is performed the localization with the odometry measures and with  
    q_current = q_start;
    P_current = P_init;
    q_time = [init_q;q_start(1:2)'];
    theta_t = [init_theta;q_start(3)];
    pixels = [init_pixels;get_pixels_coords(q_start(1), q_start(2))];
    
    rate = 1; % 1 hz is the sampling time
    sampling_time = 1/rate;

    d = 0;
    error = sqrt((q_end(1)-q_current(1))^2 + (q_end(2)-q_current(2))^2);
    Kv = 19*10^(-2)/error;
    while vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4) > 5  % meters to stop
          old_value = vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4);


%         if vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4) < 5
%             Kv = Kv/100;
%         end

        [v, gamma] = Rover_commands(q_end, q_current, Kv, Kh); % The velocity is only feedback driven, thus it will be the maximum when it is at the begininning.
        velocity_t = [velocity_t; v];
        omega = (v/L)*tan(gamma);
        commands = [v; omega];
        
        if v > 20*10^(-2)
            disp('aborted')
            break
        end

        [q_current_est, P_current_est] = update_configuration_estimates(q_current, P_current, commands, rate, odometric_noise);

        if kalman == 1
            [q_current, P_current] = EKF_(q_current_est, P_current_est, xLM, yLM, instr_var_noise, instr_noise, maxlen);
        else
            q_current = q_current_est;
            P_current = P_current_est;
        end

        value = vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4);

        if value > old_value
            break
        end
        time = time + sampling_time;
            
        theta_t = [theta_t; q_current(3)];
        theta_d_t = [theta_d_t; commands(2)];
        q_time = [q_time; [q_current(1) q_current(2)]];
        pixels = [pixels; get_pixels_coords(round(q_current(1)), round(q_current(2)))];
    
      

        if d == 5000 || d == 0
            disp(vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4))
            plot_trajectory(map, q_time, limits)
            d = 0;

        end
        d = d + 1;
    end
end


