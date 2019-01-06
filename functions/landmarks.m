function [p0,p1,X,running] = landmarks(S,p0,p1,Pose_0)
    
    global NumTrials
    global DistanceThreshold
    global InlierPercentage
    global Confidence
    
    %Calculate Fundamental matrix and inliers
    [F,inliersIndex,status] = estimateFundamentalMatrix(p0,...
    p1,'Method','RANSAC',...
    'NumTrials',NumTrials,'DistanceThreshold',DistanceThreshold,...
    'InlierPercentage',InlierPercentage);

    inlierpercentage = length(find(inliersIndex>0))/length(p0);
    
    S.t1.P=S.t1.P(inliersIndex,:);
    S.t0.P=S.t0.P(inliersIndex,:);
    if(~isempty(S.t1.X))
        S.t1.X=S.t1.X(inliersIndex,:);
    end
    disp(['inlier [%] estimateFM: ',num2str(inlierpercentage)]);
    disp(['points after estimateFM: ',num2str(length(S.t1.P))]);
    
    %Reject outliers
    inliersIndex = find(inliersIndex>0);
    p0 = p0(inliersIndex,:);
    p1 = p1(inliersIndex,:);

    %Calculate relative pose
    [R,T_transp] = relativeCameraPose(F,S.K,S.K,p0,p1);
    T = T_transp;
    
    T_temp = [R,T';0 0 0 1] * [S.t0.Pose;0 0 0 1];
    S.t1.Pose = T_temp(1:3,1:4);
    
    %Triangulate landmarks
    [p0,p1,X,running] = triLndCont(S,R,T,p0,p1);

end