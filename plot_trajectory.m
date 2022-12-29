function [none] = plot_trajectory(map, configurations, limits)
    figure()
    imshow(map,'XData',limits(1, :),'YData',limits(2, :));
    set(gca,'Ydir','normal')
    hold on
    plot(configurations(:,1), configurations(:,2), 'yo','markersize',5,'linewidth',2)