function [S,running] = contTra(pointTracker,S,prev_image,image,K)

    
    global detectNewLnd
    global cont_rescale
    running = true;
    
    %flip to get x,y
    keypoints = [S.ti.X(:,4),S.ti.Y(:,4)]*cont_rescale;
    track_img_prev = imresize(prev_image, cont_rescale);
    track_img = imresize(image, cont_rescale);
    %initialize pointtracker
    initialize(pointTracker,keypoints,track_img_prev);

    %point tracking
    [keypoints,keep] = pointTracker(track_img);
    keep_idx = find(keep>0);
    %flip to get y,x
    S.t1.P = keypoints/cont_rescale;
    S.ti.X(:,4)=keypoints(:,1)/cont_rescale;
    S.ti.Y(:,4)=keypoints(:,2)/cont_rescale;
    % check if keypoints are close together, incase delete them
    % how to choose, which keypoint to delete, if they are close together?
    [S, running] = deletecloseFt(S, keep); 
    
    %release point tracker, such that keypoints are only used from frame to
    %frame
    release(pointTracker);
    
    %Calculate pose
    [S, running] = estPose(S,K,0);
    
    if(running && detectNewLnd == true)
        %Get new features
        S = contFt(S,image,K);
    end
end