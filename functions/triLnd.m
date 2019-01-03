function [p0,p1,X] = triLnd(S,R,T,p0,p1,firstPose)

    global MaxReprojError
    
    %Strange Camera matrix shit for matlab (see documentation of
    %cameraMatrix)
    R_c = R';
    T_c = -T*R_c;
    
    camM1 = cameraMatrix(S.K,eye(3),[0 0 0]);
    camM2 = cameraMatrix(S.K,R_c,T_c);
    
    %correct pose
    %Triangulate points
    [worldP,reprojectionErrors] = triangulate(p0,p1,camM1,camM2);
    
    %transform points back into original coorinate system    
    T_mat = [firstPose; zeros(1,3) 1];
    worldP_temp = inv(T_mat)*[worldP';ones(1,size(worldP,1))];
    worldP = worldP_temp(1:3,:)';
    
    %Reject outliers
    idx_keep = find(reprojectionErrors < MaxReprojError);    
    X = worldP(idx_keep,:);
    p0 = p0(idx_keep,:);
    p1 = p1(idx_keep,:);
    %Reject points triangulated behind camera
    idx_keep = find(X(:,3)>0);
    X = X(idx_keep,:);
    p0 = p0(idx_keep,:);
    p1 = p1(idx_keep,:);
    %Reject points that are too far away
    idx_keep = find(X(:,3)<50);
    X = X(idx_keep,:);
    p0 = p0(idx_keep,:);
    p1 = p1(idx_keep,:);
    
    
end