function [S,running] = continous_tracking(pointTracker,S,prev_image,image,K)
    running = true;
    %flip to get x,y
    keypoints = fliplr(S.t1.P);
    
    %initialize pointtracker
    initialize(pointTracker,keypoints,prev_image);
    % new timestep, therefore update S
    S.t0 = S.t1;

    %point tracking
    [keypoints,keep] = pointTracker(image);
    
    %flip to get y,x
    S.t1.P = fliplr(keypoints);
    
    %release point tracker, such that keypoints are only used from frame to
    %frame
    release(pointTracker);
    
    disp(['Points that were tracked: ',num2str(length(keep(keep>0)))])
    S.t1.P = double(round(S.t1.P(find(keep>0),:)));
    
    % delete keypoints and landmarkes that are discarded
    S.t0.P = S.t0.P(find(keep>0),:);
    S.t0.X = S.t0.X(find(keep>0),:);

    % break here if less than 15 keypoints are tracked
    if length(keep(keep>0))<15
        disp('Less than 15 keypoints tracked, execution stopped')
        running=false;
    end
    
    %Calculate pose
    S = estimaterelativePose(S, K);
    
    % attempt to recover scale factor
    %recover scaling factor
    %diff_old = diff(S{2,2});
    %diff_new = diff(S{1,2});
    %distances_old = sqrt(sum(diff_old.^2,2));
    %distances_new = sqrt(sum(diff_new.^2,2));
    %median_scale = median(distances_old./distances_new)
    %S{1,6}(1:3,4) =  S{1,6}(1:3,4)*median_scale;
    %S{1,6}
    %Position(:,i) = S{2,6}(1:3,4)+S{1,6}(1:3,4);
end