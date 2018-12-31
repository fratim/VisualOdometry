function [S, running] = estPose(S,K,isBoot)
    
    global NumTrials
    global DistanceThreshold
    global InlierPercentage
    global Confidence

    running = true;
        
    % inliersIndex and status can be used for debugging, no use in normal
    % mode
    
    idx_1=find(~isnan(S.ti.X(:,1)));
    idx_2=find(~isnan(S.ti.X(:,2)));
    idx_3=find(~isnan(S.ti.X(:,3)));
    idx_4=find(~isnan(S.ti.X(:,4)));
    freq_idx = intersect(intersect(intersect(idx_1,idx_2),idx_3),idx_4);
    p0 = [S.ti.X(freq_idx,1) S.ti.Y(freq_idx,1)];
    p1 = [S.ti.X(freq_idx,4) S.ti.Y(freq_idx,4)];
    [E,inliersIndex,status] = estimateEssentialMatrix(p0,...
    p1,S.K,...
    'MaxNumTrials',NumTrials,'MaxDistance',DistanceThreshold,...
    'Confidence',Confidence);
    inliersIndex = find(inliersIndex>0);
    outliersIndex = freq_idx(find(inliersIndex==0));
    p0 = p0(inliersIndex,:);
    p1 = p1(inliersIndex,:);
    stay_idx = setdiff(1:size(S.ti.X,1),outliersIndex);
    S.ti.X=S.ti.X(stay_idx,:);
    S.ti.Y=S.ti.Y(stay_idx,:);
    %S.t1.X = S.t1.X(inliersIndex,:);
    %S.t0.X = S.t0.X(inliersIndex,:);
    [R,T_transp] = relativeCameraPose(E,S.K,p0,p1);
    T = T_transp';
    
    T_guess = [S.t0.Pose;zeros(1,3),1]*[R,T;zeros(1,3),1];
    S.t1.Pose = T_guess(1:3,1:4);
    
    %S = triangulateLandmarkslinear(S, K);
    if(isBoot==1)
        [S, running] = triLndCont(S,K,R,T,freq_idx(inliersIndex),isBoot);
    end
    
    if (isBoot==0)
        %recover scale factor
%         rand_idx = randperm(length(S.t0.X));
%         landmarks_old = S.t0.X(rand_idx,:);
%         landmarks_new = S.t1.X(rand_idx,:);
%         diff_old = diff(landmarks_old);
%         diff_new = diff(landmarks_new);
%         distances_old = sqrt(sum(diff_old.^2,2));
%         distances_new = sqrt(sum(diff_new.^2,2));
%         scales = distances_old./distances_new;
%         median_scale = median(scales);
        median_scale = 1;
        
        T_new = [S.t0.Pose;zeros(1,3),1]*[R,median_scale*T;zeros(1,3),1];
        S.t1.Pose = T_new(1:3,1:4);
        
        % triangulate landmarks with correct camera position 
        
        [S, running] = triLndCont(S,K,R,median_scale*T,freq_idx(inliersIndex),isBoot);
        
       
    end
    
    if(~running)
        return
    end
    
     % print messages
    disp('Current R,T, current estimated Pose')
    disp([R,T,S.t1.Pose])
    
end