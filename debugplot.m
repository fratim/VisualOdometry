function debugplot(S, imga, imgb)
    
    % plot only specific points
    pplot = 1:length(S.t0.P(:,1));
    % pplot = [1,2,3,4];

    subplot(2,2,1)
    imshow(imga)
    hold on
    scatter(S.t0.P(pplot,2),S.t0.P(pplot,1),'red')
    % removed for computation reasons
    %title('img0 matched features')

    subplot(2,2,2)
    imshow(imgb)
    %plot matches
    hold on
    scatter(S.t1.P(pplot,2),S.t1.P(pplot,1),'blue')
    % removed for computation reasons
    %title('img1 matched features')
    
    if(~isempty(S.t1.C))
        %plot candidates, these are saved as x y and not u v, this should
        %be changed
        hold on
        scatter(S.t1.C(:,1),S.t1.C(:,2),'green','x')
    end

    % removed for computation reasons
%     subplot(2,2,3)
%     scatter3(S.t1.X(pplot,1),S.t1.X(pplot,2),S.t1.X(pplot,3),'blue')
%     title('world points')
% 
%     %attempt to plot camera positions, not really nice yet
%     hold on
%     cam1 = plotCamera('Location',S.t1.Pose(1:3,4),'Orientation',S.t1.Pose(1:3,1:3),'Size',10);
%     hold on
%     cam2 = plotCamera('Location',S.t0.Pose(1:3,4),'Orientation',S.t0.Pose(1:3,1:3),'Size',10);
    
end