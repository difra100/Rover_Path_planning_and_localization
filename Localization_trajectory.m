function [q_time, theta_t, time, P_current, covar_det, counter] = Localization_trajectory(q_start, q_end, P_init, covar_det, odometric_noise, Kh, L, map, limits, init_q, velocity_t, init_theta, theta_d_t, time, xLM, yLM, instr_var_noise, instr_noise, maxlen, kalman, counter)
    % This function belongs to the task 3 part of the homework. It is performed the localization with the odometry measures and with  
    q_current = q_start;
    q_time = init_q;
    theta_t = init_theta;
    covar_det(counter, :) = sqrt(det(P_init));
    P_current = P_init;

    rate = 1; % 1 hz is the sampling time

    starting_rate = 10; % 10 hz is the rate of the desired configurations.

    sampling_time = 1/rate;

    d = 0;
    num = 1;
    while num < size(velocity_t, 1)% meters to stop
        v = velocity_t(num,:);
        omega = theta_d_t(num,:); 
       
        
        
        
        commands = [v; omega];
%         q_current
%         P_current
        [q_current_est, P_current_est] = update_configuration_estimates(q_current, P_current, commands, rate, odometric_noise);

        if kalman == 1
            [q_current, P_current] = EKF_(q_current_est, P_current_est, xLM, yLM, instr_var_noise, instr_noise, maxlen);
        else
            q_current = q_current_est;
            P_current = P_current_est;
        end


        time = time + sampling_time;
            
        theta_t(counter+1, :) = q_current(3);

        q_time(counter+1, :) = [q_current(1) q_current(2)];
        covar_det(counter+1, :) = sqrt(det(P_current));


        num = num+starting_rate;

        if d == 100000 || d == 0
            disp(vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4))
            plot_trajectory(map, q_time(1:counter, :), limits)
            d = 0;

        end
        d = d + 1;
        counter = counter +1;
    end
end


