function [q, P] = EKF_(q_est, P_est, xLM, yLM, instr_var_noise, instr_noise, maxlen)
    landmarks = [xLM yLM];
    rob_pos = [q_est(1) q_est(2)].*ones(size(landmarks));
    relative_dist = sqrt((rob_pos(:,1)-landmarks(:,1)).^2 + (rob_pos(:,2)-landmarks(:,2)).^2); % Check the robot conf from all the landmarks, and pick the one that has length less than maxlen.

    des_indices = find(relative_dist <= maxlen); % Sensor cannot see landmarks that are further than this maxlen.

    visible_landmarks = landmarks(des_indices, :);
    
    if size(visible_landmarks,1) == 0

        % If no landmarks are visible it is no possible to deploy the
        % kalman filter.
        q = q_est;
        P = P_est;
        return
    end
    h = zeros(2*size(visible_landmarks,1),1);

    H = zeros(2*size(visible_landmarks,1),size(q_est,1)); % Jacobian of the configurations
    Hw = zeros(2*size(visible_landmarks,1),size(instr_noise,1)); % Jacobian of the instrument noises
    count = 0;

    for i = 1:size(visible_landmarks,1)
        H_l = get_H(visible_landmarks(i,:), [q_est(1); q_est(2)]);
        h_l = get_h(visible_landmarks(i,:), q_est);

        h(count+1:count+2, :) = h_l;
        H(count+1:count+2, :) = H_l;
        Hw(count+1:count+2, :) = eye(2);
        count = count + 2;

    end


    std_vec = repmat(instr_noise,size(visible_landmarks,1),1);
    noise_vec = randn(size(std_vec,1),1).*std_vec;

    z = h + noise_vec; % Stack multiple noise vectors.
    
    K = P_est*H'/(H*P_est*H' + Hw*instr_var_noise*Hw'); % Kalman gain matrix

    q = q_est + K*(z-h);
  
    P = (eye(size(q_est,1)) - K*H)*P_est;

end


function [H_l] = get_H(pl, pk)
    % This function computes the H jacobian matrix for the range and
    % bearing angle observation. 

    H_l = [(pk(1)-pl(1))/sqrt((pk(1)-pl(1))^2 + (pk(2)-pl(2))^2) (pk(2)-pl(2))/sqrt((pk(1)-pl(1))^2 + (pk(2)-pl(2))^2) 0;
           -(pk(2)-pl(2))/((pk(1)-pl(1))^2 + (pk(2)-pl(2))^2) (pk(1)-pl(1))/((pk(1)-pl(1))^2 + (pk(2)-pl(2))^2)       -1];

end

function [h_l] = get_h(pl, qk)
    % This function computes the theoretical observation of a landmark, in
    % term of range and bearing angle.
    h_l = [sqrt((qk(1)-pl(1))^2 + (qk(2)-pl(2))^2);
           atan2(pl(2)-qk(2),pl(1)-qk(1)) - qk(3)];
end

