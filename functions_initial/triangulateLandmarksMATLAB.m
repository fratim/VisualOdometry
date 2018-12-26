function S = triangulateLandmarksMATLAB(S, K)


    % K inverse to meet matlab convention
    cameraParams = cameraParameters('IntrinsicMatrix',K');
    stereoParams = stereoParameters(cameraParams,cameraParams,...
                                    S{1,6}(1:3,1:3),S{1,6}(1:3,4));
    
    pt1 = S{2,1};
    pt2 = S{1,1};
    
    [worldP,reprojectionErrors] = triangulate(pt1,pt2,stereoParams);
    
    %attempt: discard landmarks and according keypoitns, if reprojection
    %error is large (is this in pixels?) just trying with 1, looks fine
    %problem: some points are still projected behind the damn camera
    
    idx_keep = find(reprojectionErrors<2);
    worldP = worldP(idx_keep,:);
    
    %discard according feature points
    S{1,1} = S{1,1}(idx_keep,:);
    S{2,1} = S{2,1}(idx_keep,:);
    
    
    S{1,2} = worldP;
end