function [none] = plot_trajectory(map, configurations, limits, color)
    figure()
    Xvec = limits(1,:);
    Yvec = limits(2,:);
    imshow(map,'XData',Xvec,'YData',Yvec);
    set(gca,'Ydir','normal')
    axis on
    grid on
    xlabel('X')
    ylabel('Y')
    hold on 
    plot(configurations(:,1), configurations(:,2), color,'markersize',5,'linewidth',2)