function [S, running] = estPose(S, K, isBoot)
    
    global NumTrials
    global DistanceThreshold
    global InlierPercentage
    global Confidence

    % inliersIndex and status can be used for debugging, no use in normal
    % mode
    [F,inliersIndex,status] = estimateFundamentalMatrix(round(fliplr(S.t0.P)),...
    round(fliplr(S.t1.P)),'Method','RANSAC',...
    'NumTrials',NumTrials,'DistanceThreshold',DistanceThreshold,'InlierPercentage',...
    InlierPercentage,'Confidence',Confidence);

    [R,T_transp] = relativeCameraPose(F,S.K,fliplr(S.t0.P),fliplr(S.t1.P));
    T = T_transp';
    
    T_guess = [S.t0.Pose;zeros(1,3),1]*[R,T;zeros(1,3),1];
    S.t1.Pose = T_guess(1:3,1:4);
    
    %S = triangulateLandmarkslinear(S, K);
    [S, running] = triLndCont(S,K,R,T, isBoot);
    
    if(~running)
        return
    end
    
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
        median_scale = median(scales);
        T_new = [S.t0.Pose;zeros(1,3),1]*[R,median_scale*T;zeros(1,3),1];
        S.t1.Pose = T_new(1:3,1:4);
        
        % triangulate landmarks with correct camera position 
        S = triLndCont(S,K,R,median_scale*T,isBoot);
        
        % print messages
        disp('Current R,T, current estimated Pose')
        disp([R,T,S.t1.Pose])
    end
    
end