function S = triangulateLandmarksMATLAB(S,K,R,T, isBoot)

    run ParkingParameters

    % K inverse to meet matlab convention
    cameraParams = S.K;
    stereoParams = stereoParameters(cameraParams,cameraParams,...
                                    R,-T);
    
    ptold = fliplr(S.t0.P);
    ptnew = fliplr(S.t1.P);
    
    [worldP,reprojectionErrors] = triangulate(ptold,ptnew,stereoParams);
    
    %attempt: discard landmarks and according keypoitns, if reprojection
    %error is large (is this in pixels?) just trying with 1, looks fine
    %problem: some points are still projected behind the damn camera
    
    idx_keep = find(reprojectionErrors<MaxReprojError);
    worldP = worldP(idx_keep,:);
    
    % break here if less than 15 keypoints are tracked
    if length(idx_keep(idx_keep>0))<15
        disp('Less than 15 keypoints tracked, execution stopped')
        return
    end
    
    % discard according feature points and landmarks
    S.t0.P = S.t0.P(idx_keep,:);
    S.t1.P = S.t1.P(idx_keep,:);
    if (isBoot == false)
        S.t0.X = S.t0.X(idx_keep,:);
    end
    S.t1.X = worldP;
end