function X = triangulateNewLandmarklinear(p1,p2,P1,P2,cameraParams)
    X=[];
    R = P1(1:3,1:3)'*P2(1:3,1:3);
    %R = P2(1:3,1:3)-P1(1:3,1:3);
    T = P2(1:3,4)-P1(1:3,4);
    % K inverse to meet matlab convention
    stereoParams = stereoParameters(cameraParams,cameraParams,...
                                    R,T);
    [worldP,reprojectionErrors] = triangulate(p1,p2,stereoParams);
    
    %attempt: discard landmarks and according keypoitns, if reprojection
    %error is large (is this in pixels?) just trying with 1, looks fine
    %problem: some points are still projected behind the damn camera
    idx_keep = find(reprojectionErrors<0.25);
    X = worldP(idx_keep,:);
   
end