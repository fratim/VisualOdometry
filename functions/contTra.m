function [S,running, scale_fac] = contTra(pointTracker,S,prev_image,image,K, scale_fac)
    
    global MinPoints
    
    running = true;
    
    %flip to get x,y
    keypoints = fliplr(S.t1.P);
    
    %initialize pointtracker
    initialize(pointTracker,keypoints,prev_image);
    
    % new timestep, therefore update S
    S.t0 = S.t1;

    %point tracking
    [keypoints,keep] = pointTracker(image);
    
    % check if keypoints are close together, incase delete them
    % how to choose, which keypoint to delete, if they are close together?
    [S, keep] = deletecloseFt(S, keep); 
    
    %flip to get y,x
    S.t1.P = fliplr(keypoints(find(keep>0),:));
    
    % delete keypoints and landmarkes that are discarded
    S.t0.P = S.t0.P(find(keep>0),:);
    S.t0.X = S.t0.X(find(keep>0),:);
    
    % break here if less than Minpoints keypoints are tracked
    if length(keep(keep>0)) < MinPoints
        disp('Less than Minpoints keypoints tracked, execution stopped')
        running=false;
    end
    
    %release point tracker, such that keypoints are only used from frame to
    %frame
    release(pointTracker);
    
    %Calculate pose
    S = estPose(S,K,0);
    
    %Get new features
    S = contFt(S,image,K);
end