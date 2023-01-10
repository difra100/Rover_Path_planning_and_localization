function [q, P] = update_configuration_estimates(q_old, P_old, commands, rate, noise)
    % INPUTs: q_old: Previous configuration, P_old: Previous Co-variance
    % matrix, commands: driving and steering velocity, noise: odometrix
    % noises in matrix form, kalman: This is a flag to say if compute or
    % not the kalman adjustment of a measure.
    % OUTPUTs: q: Next configuration after the update, P: New configuration
    % of the co-variance matrix.

    sampling_time = 1/rate;
    delta_d = sampling_time*commands(1); % travelled distance
    delta_theta = sampling_time*commands(2); % heading angle change
    dist_noise = noise(1,1);

    orient_noise = noise(2,2);

    % Partial derivatives

    % Jacobian of the configurations
    Fq = [1 0 -delta_d*sin(q_old(3));
          0 1 delta_d*cos(q_old(3));
          0 0                     1];

    % Jacobian of the noises
    Fv = [cos(q_old(3)) 0;
          sin(q_old(3)) 0;
                      0 1];

    q = zeros(size(q_old));

    %  'Integration step': Estimate of the configurations 
    q(1,:) = q_old(1,:) + (delta_d+dist_noise)*cos(q_old(3,:));
    q(2,:) = q_old(2,:) + (delta_d+dist_noise)*sin(q_old(3,:));
    q(3,:) = q_old(3,:) + delta_theta + orient_noise;

    %  'Co-variance matrix estimate'
    P = Fq*P_old*Fq' + Fv*noise*Fv';

    

end