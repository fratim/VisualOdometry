function [S, running] = triLndCont(S,K,R,T, isBoot)

    global MaxReprojError
    global MinPoints

    running = true;
    
    % K inverse to meet matlab convention
    cameraParams = S.K;
    stereoParams = stereoParameters(cameraParams,cameraParams,...
                                    R,-T);
    ptold = [S.ti.X(:,1) S.ti.Y(:,2)];
    ptnew = [S.ti.X(:,4) S.ti.Y(:,4)];
    
    [worldP,reprojectionErrors] = triangulate(ptold,ptnew,stereoParams);
    %horizon_idx = find(abs(worldP(:,3))<80);
    
    %ptold = ptold(horizon_idx,:);
    %ptnew = ptnew(horizon_idx,:);
    %S.t0.P=S.t0.P(horizon_idx,:);
    %S.t1.P=S.t1.P(horizon_idx,:);
    
    %[worldP,reprojectionErrors] = triangulate(ptold,ptnew,stereoParams);
    idx_keep = find(reprojectionErrors < MaxReprojError);
    worldP = worldP(idx_keep,:);
    % break here if less than 15 keypoints are tracked
    if length(idx_keep(idx_keep>0)) < MinPoints
        disp('Less than MinPoints points tracked, triangulation error too large!')
        running = false;
        return
    end
    
    % discard according feature points and landmarks
%     S.t0.P = S.t0.P(idx_keep,:);
%     S.t1.P = S.t1.P(idx_keep,:);
%     S.ti.X = S.ti.X(idx_keep,:);
%     S.ti.Y = S.ti.Y(idx_keep,:);
    if (isBoot == false)
        %S.t0.X = S.t0.X(idx_keep,:);
    end
    S.t1.X = worldP;
end