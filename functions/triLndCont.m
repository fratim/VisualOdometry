function [p0,p1,X,running] = triLndCont(S,R,T,p0,p1)

    global MaxReprojError
    global MinPoints
    
    running = true;
    
    % K inverse to meet matlab convention
    cameraParams = S.K;
    
    %Strange Camera matrix shit for matlab (see documentation of
    %cameraMatrix)
    R_c = R';
    T_c = -T*R_c;
    
    camM1 = cameraMatrix(S.K,eye(3),[0 0 0]);
    camM2 = cameraMatrix(S.K,R_c,T_c);
    
    %Triangulate points
    [worldP,reprojectionErrors] = triangulate(p0,p1,camM1,camM2);
    
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
    
    % break here if less than 15 keypoints are tracked
    if length(idx_keep) < MinPoints
        disp('Less than MinPoints points tracked, triangulation error too large!')
        running = false;
        return
    end
end