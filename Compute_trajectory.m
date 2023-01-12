function [q_time, velocity_t, theta_t, theta_d_t, time, num_next] = Compute_trajectory(q_start, q_end, Kh, L, map, limits, init_q, velocity_t, init_theta, theta_d_t, time, num, rate)

    q_current = q_start;
    q_time = init_q;
    q_time(num, :) = q_start(1:2)';
    theta_t = init_theta;
    theta_t(num) = q_start(3);
    
    
    sampling_time = 1/rate;

    d = 0;
    
    while vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4) > 1  % meters to stop
        old_value = vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4);
        error = sqrt((q_end(1)-q_current(1))^2 + (q_end(2)-q_current(2))^2);
        Kv = 19.5*10^(-2)/error;


        [v, gamma] = Rover_commands(q_end, q_current, Kv, Kh); % The velocity is only feedback driven, thus it will be the maximum when it is at the begininning.
        velocity_t(num) =  v;

        commands = [v; gamma];
        
        if v > 20*10^(-2)
            disp('aborted')
            break
        end
        t_span = [time time+sampling_time];

        [q_current, variation] = update_kinematic_model(q_current, commands, L, t_span);


        value = vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4);

        if value > old_value
            disp('Error:')
            disp(vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4))
            break
        end
        time = time + sampling_time;
            
        theta_t(num+1) = q_current(3);
        theta_d_t(num) = variation(3);
        q_time(num+1, :) = [q_current(1) q_current(2)];
    
        num = num+1;

        if d == 100000 || d == 0
            disp(vpa(norm([q_current(1);q_current(2)]-[q_end(1); q_end(2)]),4))
            plot_trajectory(map, q_time(1:num+1,:), limits, 'r')
            hold on
            d = 0;

        end
        d = d + 1;
    end
    num_next = num;
end











