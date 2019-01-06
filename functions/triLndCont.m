function [p0,p1,X,running] = triLndCont(S,R,T,p0,p1)

    global MaxReprojError
    global MinPoints
    
    running = true;
    
    %Strange Camera matrix shit for matlab (see documentation of
    %cameraMatrix)
    
    R_n = S.t1.Pose(1:3,1:3);
    T_n = S.t1.Pose(1:3,4)';
    
    R_o = S.t0.Pose(1:3,1:3);
    T_o = S.t0.Pose(1:3,4)';
    
    R_0= R_o';
    T_0= -T_o*R_0;
   
    R_1= R_n';
    T_1= -T_n*R_1;
    
    R_c = R';
    T_c = -T*R_c;
    
    
    R_c = R';
    T_c = -T*R_c;
    
    camM1 = cameraMatrix(S.K,R_0,T_0);
    camM2 = cameraMatrix(S.K,R_1,T_1);
    
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
%     %Reject points triangulated behind camera
%     idx_keep = find(X(:,3)>0);
%     X = X(idx_keep,:);
%     p0 = p0(idx_keep,:);
%     p1 = p1(idx_keep,:);
%     %Reject points that are too far away
%     idx_keep = find(X(:,3)<40);
%     X = X(idx_keep,:);
%     p0 = p0(idx_keep,:);
%     p1 = p1(idx_keep,:);
    
    % break here if less than 15 keypoints are tracked
    if length(idx_keep) < MinPoints
        disp('Less than MinPoints points tracked, triangulation error too large!')
        running = false;
        return
    end
    
end