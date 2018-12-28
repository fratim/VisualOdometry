function X = triangulateNewLandmarklinear(p1,p2,P1,P2,K)
    
    R = P1(1:3,1:3)'*P2(1:3,1:3);
    T = P2(1:3,4)-P1(1:3,4);
    % K inverse to meet matlab convention
    cameraParams = cameraParameters('IntrinsicMatrix',K');
    stereoParams = stereoParameters(cameraParams,cameraParams,...
                                    R,T);
    
    ptold = p1;
    ptnew = p2;
    
    [worldP,reprojectionErrors] = triangulate(ptold,ptnew,stereoParams);
    
    %attempt: discard landmarks and according keypoitns, if reprojection
    %error is large (is this in pixels?) just trying with 1, looks fine
    %problem: some points are still projected behind the damn camera
    
    idx_keep = find(reprojectionErrors<0.25);
    X = worldP(idx_keep,:);

end