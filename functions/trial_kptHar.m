function [S_data, running] = trial_kptHar(S_data, img0, img1)

    %load parameters
    global HrScale
    %global HrKernel
    %global MatchThresholdInit
    %global HrQuality
    
    harris_patch_size = 9;
    harris_kappa = 0.08;
    num_keypoints = 1500;
    nonmaximum_supression_radius = 8;
    descriptor_radius = 9;
    match_lambda = 3;
    
    running = true;
    
    %resize images
    img0 = imresize(img0, HrScale);
    img1 = imresize(img1, HrScale);
    
    scores_0 = harris(img0, harris_patch_size, harris_kappa);
    kp_0 = selectKeypoints(...
        scores_0, num_keypoints, nonmaximum_supression_radius);
    desc_0 = describeKeypoints(img0, kp_0, descriptor_radius);
    
    scores_1 = harris(img1, harris_patch_size, harris_kappa);
    kp_1 = selectKeypoints(...
        scores_1, num_keypoints, nonmaximum_supression_radius);
    desc_1 = describeKeypoints(img1, kp_1, descriptor_radius);
    
    matches = matchDescriptors(desc_1, desc_0, match_lambda);
    
    kp_0 = fliplr(kp_0');
    kp_1 = fliplr(kp_1');
    
    matchedPoints1 = kp_0(matches(matches>0),:);
    matchedPoints2 = kp_1(find(matches>0),:);
    %debug
    showMatchedFeatures(img0,img1,matchedPoints1,matchedPoints2)
    
    S_data.t1.P=double(fliplr(matchedPoints2))./HrScale;
    S_data.t0.P=double(fliplr(matchedPoints1))./HrScale;
    
end