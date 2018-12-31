function [S,running] = contTra(pointTracker,S,prev_image,image,K)

    
    global detectNewLnd
    
    running = true;
    
    %flip to get x,y
    keypoints = [S.ti.X(:,4),S.ti.Y(:,4)];
    
    %initialize pointtracker
    initialize(pointTracker,keypoints,prev_image);

    %point tracking
    [keypoints,keep] = pointTracker(image);
    keep_idx = find(keep>0);
    %flip to get y,x
    S.t1.P = keypoints;
    S.ti.X(:,4)=keypoints(:,1);
    S.ti.Y(:,4)=keypoints(:,2);
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