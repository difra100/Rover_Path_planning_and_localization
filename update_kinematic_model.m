function [new_configuration, variation] = update_kinematic_model(q_act, commands, L, t_span)
    % INPUTs: q_act: Actual configuration vector, commands:
    % Velocity+Steering angle commands, L: Axles distance
    % OUTPUT: new_configuration: New configuration vector.

    variation = get_discrete_var(q_act, commands, L)'; % Time that is passed during the state transition.

    [t,new_configuration] = ode45(@(t,new_configuration) variation, t_span, q_act);
    
    new_configuration = new_configuration(end,:)';

end

function variation = get_discrete_var(q, commands, L)
    % INPUTS: q_act: Actual configuration vector, commands:
    % Velocity+Steering angle commands, L: Axles distance
    % OUTPUT: variation: Derivative of the kinematic model.
    variation(1) = commands(1)*cos(q(3));
    variation(2) = commands(1)*sin(q(3));
    variation(3) = (commands(1)/L)*tan(commands(2));

end
