function [] = plot_frame(frame,lm,traj)
    hold on
    set(gcf, 'Position',  [100, 100, 960, 540])
    pos1 = [0.05 0.05 0.2 0.4];
    subplot('Position',pos1)
    plot(0)
    axis equal
    title('# Tracked landmarks of last 20 frames')
    
    pos2 = [0.3 0.05 0.2 0.4];
    subplot('Position',pos2)
    plot(0)
    axis equal
    title('Full Trajectory')
    
    pos3 = [0.05 0.6 0.4 0.4];
    subplot('Position',pos3)
    imshow(frame)
    title('Current Frame')
    
    pos4 = [0.55 0.05 0.4 0.9];
    subplot('Position',pos4)
    plot(0)
    axis equal
    title('Trajectory of last 20 frames and landmarks')
end