function [p0,p1,X,running] = triLndCont(S,R,T,p0,p1)

    global MaxReprojError
    global MinPoints
    
    running = true;
    
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
    %hacking
    %T_mat = [eye(3), S.t0.Pose(1:3,4); zeros(1,3) 1];
    %worldP_temp = T_mat*[worldP';ones(1,length(worldP))];
    %worldP = worldP_temp(1:3,:)';
    
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
    idx_keep = find(X(:,3)<200);
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