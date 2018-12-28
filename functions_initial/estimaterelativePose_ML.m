function S = estimaterelativePose_ML(S, K, isBoot)
    
    % inliersIndex and status can be used for debugging, no use in normal
    % mode
    [F,inliersIndex,status] = estimateFundamentalMatrix(fliplr(S.t0.P(:,:)),...
    fliplr(S.t1.P(:,:)),'Method','RANSAC',...
    'NumTrials',100000,'DistanceThreshold',0.001,'InlierPercentage',80,'Confidence',99.99);

    cameraParams = cameraParameters('IntrinsicMatrix',K');
    [R,T_transp] = relativeCameraPose(F,cameraParams,fliplr(S.t0.P),fliplr(S.t1.P));
    T = T_transp';
    
    disp('R and T estimated over last iteration: ')
    disp([R,T])
 
    T_guess = [S.t0.Pose;zeros(1,3),1]*[R,T;zeros(1,3),1];
    S.t1.Pose = T_guess(1:3,1:4);
    
    %S = triangulateLandmarkslinear(S, K);
    S = triangulateLandmarksMATLAB(S,K,R,T, isBoot);
    
    if (isBoot==0)
        %recover scale factor
        rand_idx = randperm(length(S.t0.X));
        landmarks_old = S.t0.X(rand_idx,:);
        landmarks_new = S.t1.X(rand_idx,:);
        diff_old = diff(landmarks_old);
        diff_new = diff(landmarks_new);
        distances_old = sqrt(sum(diff_old.^2,2));
        distances_new = sqrt(sum(diff_new.^2,2));
        scales = distances_old./distances_new;
        
        median_scale = median(scales)
        if(isnan(median_scale))
            median_scale;
        end
        T_new = [S.t0.Pose;zeros(1,3),1]*[R,median_scale*T;zeros(1,3),1];
        S.t1.Pose = T_new(1:3,1:4);
        
        % triangulate landmarks with correct camera position 
        S = triangulateLandmarksMATLAB(S,K,R,median_scale*T,isBoot);
    end
    
end