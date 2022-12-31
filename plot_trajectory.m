function [none] = plot_trajectory(map, configurations, Xvec, Yvec)
    figure()
    imshow(map,'XData',Xvec,'YData',Yvec);
    set(gca,'Ydir','normal')
    hold on
    plot(configurations(:,1), configurations(:,2), 'yo','markersize',5,'linewidth',2)