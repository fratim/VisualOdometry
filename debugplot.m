function debugplot(S, imga, imgb)
    
    % plot only specific points
    pplot = 1:length(S{2,1}(:,1));
    % pplot = [1,2,3,4];

    subplot(2,2,1)
    imshow(imga)
    hold on
    scatter(S{2,1}(pplot,2),S{2,1}(pplot,1),'red')
    title('img0 matched features')

    subplot(2,2,2)
    imshow(imgb)
    hold on
    scatter(S{1,1}(pplot,2),S{1,1}(pplot,1),'blue')
    title('img1 matched features')

    subplot(2,2,3)
    scatter3(S{1,2}(pplot,1),S{1,2}(pplot,2),S{1,2}(pplot,3),'blue')
    title('world points')

    %attempt to plot camera positions, not really nice yet
    hold on
    cam1 = plotCamera('Location',S{1,6}(1:3,4),'Orientation',S{1,6}(1:3,1:3),'Size',10);
    hold on
    cam2 = plotCamera('Location',S{2,6}(1:3,4),'Orientation',S{2,6}(1:3,1:3),'Size',10);
    
end