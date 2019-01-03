function [p0,p1,X] = triLnd(S,R,T,p0,p1,firstPose)

    global MaxReprojError
    global NumTrials
    global DistanceThreshold
    global InlierPercentage
    global Confidence
    
    %Calculate Fundamental matrix and inliers
    try
    [F,inliersIndex,status] = estimateFundamentalMatrix(p0,...
    p1,'Method','RANSAC',...
    'NumTrials',NumTrials,'DistanceThreshold',DistanceThreshold);
    catch 
        p0=[];
        p1=[];
        X = [];
        return
    end
    p0=p0(inliersIndex,:);
    p1=p1(inliersIndex,:);
    inlierpercentage = length(find(inliersIndex>0))/length(p0);
    %Strange Camera matrix shit for matlab (see documentation of
    %cameraMatrix)
    R_c = R';
    T_c = -T*R_c;
    
    camM1 = cameraMatrix(S.K,eye(3),[0 0 0]);
    camM2 = cameraMatrix(S.K,R_c,T_c);
    
    %correct pose
    %Triangulate points
    [worldP,reprojectionErrors] = triangulate(p0,p1,camM1,camM2);
    idx_keep = find(reprojectionErrors < MaxReprojError); 
    worldP = worldP(idx_keep,:);
    p0=p0(idx_keep,:);
    p1=p1(idx_keep,:);
    idx_keep = find(worldP(:,3)>0);
    worldP = worldP(idx_keep,:);
    p0=p0(idx_keep,:);
    p1=p1(idx_keep,:);
    %idx_keep = find(worldP(:,3)<40);
    %worldP = worldP(idx_keep,:);
    %p0=p0(idx_keep,:);
    %p1=p1(idx_keep,:);
    
    %transform points back into original coorinate system    
    T_mat = [firstPose; zeros(1,3) 1];
    
    worldP_temp = T_mat*[worldP';ones(1,size(worldP,1))];
    worldP = worldP_temp(1:3,:)';
    X=worldP;
    %Reject outliers
       
%     X = worldP(idx_keep,:);
%     p0 = p0(idx_keep,:);
%     p1 = p1(idx_keep,:);
%     %Reject points triangulated behind camera
%     
%     %Reject points that are too far away
%     idx_keep = find(X(:,3)<50);
%     X = X(idx_keep,:);
%     p0 = p0(idx_keep,:);
%     p1 = p1(idx_keep,:);
    
    
end