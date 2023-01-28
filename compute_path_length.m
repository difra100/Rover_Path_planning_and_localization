function l = compute_path_length(path_q)
    % path_q: configuration x 2, (2: (x,y))
    l = 0;
    prev = path_q(1,:);
    for i = 2:size(path_q,1)
        inter_dist = sqrt((path_q(i,1)-prev(:,1))^2+(path_q(i,2)-prev(:,2))^2);
        l = l+inter_dist;
        prev = path_q(i,:);

    end

end
