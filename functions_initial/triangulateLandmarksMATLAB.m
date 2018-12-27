function S = triangulateLandmarksMATLAB(S,K,R,T, isBoot)


    % K inverse to meet matlab convention
    cameraParams = cameraParameters('IntrinsicMatrix',K');
    stereoParams = stereoParameters(cameraParams,cameraParams,...
                                    R,T);
    
    ptold = fliplr(S.t0.P);
    ptnew = fliplr(S.t1.P);
    
    [worldP,reprojectionErrors] = triangulate(ptold,ptnew,stereoParams);
    
    %attempt: discard landmarks and according keypoitns, if reprojection
    %error is large (is this in pixels?) just trying with 1, looks fine
    %problem: some points are still projected behind the damn camera
    
    idx_keep = find(reprojectionErrors<0.25);
    worldP = worldP(idx_keep,:);
    
    % discard according feature points and landmarks
    S.t0.P = S.t0.P(idx_keep,:);
    S.t1.P = S.t1.P(idx_keep,:);
    if (isBoot == false)
        S.t0.X = S.t0.X(idx_keep,:);
    end
    S.t1.X = worldP;
end