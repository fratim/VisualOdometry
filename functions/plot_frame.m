function [] = plot_frame(S,traj,land_hist,frame)
    hold off
    set(gcf, 'Position',  [100, 100, 960, 540])
    pos1 = [0.05 0.05 0.2 0.4];
    subplot('Position',pos1)
    plot(-19:1:0,land_hist)
    axis([-20 0 0 400])
    title('# Tracked landmarks of last 20 frames')
    
    pos2 = [0.3 0.05 0.2 0.4];
    subplot('Position',pos2)
    plot(traj(:,1),traj(:,3))
    axis equal
    title('Full Trajectory')
    
    pos3 = [0.05 0.6 0.4 0.4];
    subplot('Position',pos3)
    imshow(frame)
    title('Current Frame')
    
    pos4 = [0.55 0.05 0.4 0.9];
    subplot('Position',pos4)
    if(length(traj)>20)
        plot(0,0)
        plot(traj(end-20:end,1),traj(end-20:end,3),'*')
    else
        disp('jo')
        plot(traj(:,1),traj(:,3),'*')
        
    end
    axis equal
    title('Trajectory of last 20 frames and landmarks')
end