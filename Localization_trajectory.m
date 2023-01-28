function [q_time, theta_t, time, P_current, covar_det, P_cell, counter] = Localization_trajectory(q_start, q_end, P_init, P_cell, covar_det, odometric_noise, map, limits, init_q, old_q, init_theta, old_theta, time, xLM, yLM, instr_var_noise, instr_noise, maxlen, kalman, counter, number)
    % This function belongs to the task 3 part of the homework. It is performed the localization with the odometry measures and with  
    q_current = q_start;
    q_time = init_q;
    q_time(counter, :) = [q_current(1) q_current(2)];
    theta_t = init_theta;
    covar_det(counter, :) = sqrt(det(P_init));
    P_current = P_init;

    rate = 1; % 1 hz is the sampling time

    sampling_time = 1/rate;
 
    
    while counter < number
        

        travelled_distance = sqrt((old_q(counter+1,1)-old_q(counter,1))^2 + (old_q(counter+1,2)-old_q(counter,2))^2);
        angle_var = old_theta(counter+1,:) - old_theta(counter,:);
        
        
        commands = [travelled_distance; angle_var];

        [q_current_est, P_current_est] = update_configuration_estimates(q_current, P_current, commands, rate, odometric_noise);

        if kalman == 1

            [q_current, P_current] = EKF_([old_q(counter+1,:) old_theta(counter+1,:)]', q_current_est, P_current_est, xLM, yLM, instr_var_noise, instr_noise, maxlen);
        else

            q_current = q_current_est;
            P_current = P_current_est;
        end

        P_cell{counter+1} = P_current;

        time = time + sampling_time;
            
        theta_t(counter+1, :) = q_current(3);

        q_time(counter+1, :) = [q_current(1) q_current(2)];
        covar_det(counter+1, :) = sqrt(det(P_current));


   
        counter = counter + 1;
    end
end


