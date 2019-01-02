function [p0,p1,X,running] = landmarks(S,p0,p1,Pose_0)
    
    global NumTrials
    global DistanceThreshold
    global InlierPercentage
    global Confidence
    
    %Calculate Fundamental matrix and inliers
    [F,inliersIndex,status] = estimateFundamentalMatrix(p0,...
    p1,'Method','RANSAC',...
    'NumTrials',NumTrials,'DistanceThreshold',DistanceThreshold);
    
    %Reject outliers
    inliersIndex = find(inliersIndex>0);
    p0 = p0(inliersIndex,:);
    p1 = p1(inliersIndex,:);

    %Calculate relative pose
    [R,T_transp] = relativeCameraPose(F,S.K,S.K,p0,p1);
    T = T_transp;
    
    %Triangulate landmarks
    [p0,p1,X,running] = triLndCont(S,R,T,p0,p1);

end