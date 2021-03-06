function [] = plot_frame(S,traj,land_hist,frame)
    hold off
    set(gcf, 'Position',  [100, 100, 960, 540])
    pos1 = [0.05 0.05 0.2 0.4];
    subplot('Position',pos1)
    position_x = S.t1.Pose(1,4);
    position_z = S.t1.Pose(1,4);
    %axis([position_x-40 position_x+40 position_z-40 position_z+40]) %this does not seem to have any effect
    plot(-19:1:0,land_hist)
    axis([-19 1 0 500]);
    title('# Tracked landmarks of last 20 frames')
    
    pos2 = [0.3 0.05 0.2 0.4];
    subplot('Position',pos2)
    plot(traj(:,1),traj(:,3))
    axis equal
    title('Full Trajectory')
    
    pos3 = [0.05 0.6 0.4 0.4];
    subplot('Position',pos3)
    imshow(frame)
    hold on
    plot(S.t1.P(:,1),S.t1.P(:,2),'gx')
    if(length(S.t1.CC)>0)
        plot(S.t1.CC(:,1),S.t1.CC(:,2),'rx')
    end
    hold off
    title('Current Frame')
    
    pos4 = [0.55 0.05 0.4 0.9];
    subplot('Position',pos4)
    if(length(traj)>20)
        plot(0,0)
        scatter(S.t1.X(:,1),S.t1.X(:,3))       
        hold on
        plot(traj(end-20:end,1),traj(end-20:end,3),'*')
        hold off
    else
        disp('jo')
        scatter(S.t1.X(:,1),S.t1.X(:,3))
        hold on
        plot(traj(:,1),traj(:,3),'*')       
        hold off
        
    end
    axis equal
    title('Trajectory of last 20 frames and landmarks')
end