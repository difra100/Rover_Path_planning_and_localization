function [v, gamma] = Rover_commands(q_des, q_act, Kv, Kh)
% INPUT: q_des: [x*, y*, theta*]', q_act: [x, y, theta]', kv: control gain
% for the velocity command, kh: control gain for the steering angle
% command.
% OUTPUT: Commands for the Rover.
    error = sqrt((q_des(1)-q_act(1))^2 + (q_des(2)-q_act(2))^2);
    v = Kv*error;
    gamma = Kh*(q_des(3)-q_act(3));
    

end