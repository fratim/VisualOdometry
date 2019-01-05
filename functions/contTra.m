function [S,running] = contTra(pointTracker,S,prev_image,image)

    
    global detectNewLnd
    global cont_rescale
    running = true;
    
    %flip to get x,y
    idx_P = 1:length(S.t1.P);
    idx_CC = (1+length(S.t1.P)):(length(S.t1.CC)+length(S.t1.P));
    
    keypoints = [S.t1.P;S.t1.CC]*cont_rescale;
    track_img_prev = imresize(prev_image, cont_rescale);
    track_img = imresize(image, cont_rescale);
    %initialize pointtracker
    initialize(pointTracker,keypoints,track_img_prev);

    %point tracking
    [keypoints,keep] = pointTracker(track_img);
    
    S.t1.P = double(keypoints(idx_P,:))/cont_rescale;
    
    disp(['candidates before tracking: ', num2str(length(S.t1.CC))]);
    
    if(~isempty(idx_CC))
        % keep only keypoints that are candidates and flagged as keep
        S.t1.CC = double(keypoints(idx_CC(keep(idx_CC)),:))/cont_rescale;
        %keep only F and T for keypoints that were tracked correctly
        keep_tracking = idx_CC(keep(idx_CC))-length(idx_P);
        S.t1.C = S.t1.C(keep_tracking,:);
        S.t1.F = S.t1.F(keep_tracking,:);
        S.t1.T = S.t1.T(keep_tracking,:);
    end
    
    disp(['candidates kept after tracking: ', num2str(length(S.t1.CC))]);
    
    keep = keep(idx_P);
        
    disp(['points after KLT: ',num2str(length(find(keep>0)))]);
    
    % how to choose, which keypoint to delete, if they are close together?
    [S, running] = deletecloseFt(S, keep); 
    
    %[ymax,xmax] = size(image);
    
    %S = enforceBlocks(S,xmax,ymax);
    
    disp(['points after deletecloseFt: ',num2str(length(S.t1.P))]);
    
    %release point tracker, such that keypoints are only used from frame to
    %frame
    release(pointTracker);
    
    %Calculate pose
    [S, running] = estPose(S,0);
    
    if(running && detectNewLnd == true)
        %Get new features
        S = contFt_KLT(S,image);
    end
end