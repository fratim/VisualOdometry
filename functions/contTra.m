function [S,running] = contTra(pointTracker,S,prev_image,image,K,width,height)
    
    global detectNewLnd
    
    running = true;
    
    %flip to get x,y
    keypoints = fliplr(S.t1.P);
    
    %initialize pointtracker
    initialize(pointTracker,keypoints,prev_image);

    %point tracking
    [keypoints,keep] = pointTracker(image);
    
    %flip to get y,x
    S.t1.P = fliplr(keypoints);
    
    % check if keypoints are close together, incase delete them
    % how to choose, which keypoint to delete, if they are close together?
    [S, running] = deletecloseFt(S, keep); 
    
    %release point tracker, such that keypoints are only used from frame to
    %frame
    release(pointTracker);
    
    %Calculate pose
    [S, running] = estPose(S,K,0,width,height);
    
    if(running && detectNewLnd == true)
        %Get new features
        S = contFt(S,image,K);
    end
end