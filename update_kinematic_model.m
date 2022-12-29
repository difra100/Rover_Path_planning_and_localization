function [new_configuration, variation, int_time] = update_kinematic_model(q_act, commands, L)
    % INPUTs: q_act: Actual configuration vector, commands:
    % Velocity+Steering angle commands, L: Axles distance
    % OUTPUT: new_configuration: New configuration vector.

    variation = get_discrete_var(q_act, commands, L)';
    space = sqrt(variation(1)^2 + variation(2)^2);
    int_time = space/commands(1); % Time that is passed during the state transition.
    new_configuration = q_act + variation;


end

function variation = get_discrete_var(q, commands, L)
    % INPUTS: q_act: Actual configuration vector, commands:
    % Velocity+Steering angle commands, L: Axles distance
    % OUTPUT: variation: Derivative of the kinematic model.
    variation(1) = commands(1)*cos(q(3));
    variation(2) = commands(1)*sin(q(3));
    variation(3) = (commands(1)/L)*tan(commands(2));

end
