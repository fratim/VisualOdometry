function S_data = kptHar(S_data, img0, img1)

    %load parameters
    global HrScale
    global HrKernel
    global MatchThresholdInit
    global HrQuality
    
    %resize images
    img0 = imresize(img0, HrScale);
    img1 = imresize(img1, HrScale);
    
    points1 = detectHarrisFeatures(img0,'MinQuality',HrQuality,'FilterSize',HrKernel);
    points2 = detectHarrisFeatures(img1,'MinQuality',HrQuality,'FilterSize',HrKernel);
    
    %Extract the neighborhood features.

    [features1,valid_points1] = extractFeatures(img0,points1);
    [features2,valid_points2] = extractFeatures(img1,points2);
    
    %Match the features.

    indexPairs = matchFeatures(features1,features2,'MatchThreshold',MatchThresholdInit,'Unique',true);
    
    %Retrieve the locations of the corresponding points for each image.

    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    matchedPoints2 = valid_points2(indexPairs(:,2),:);
    
    %debug
    showMatchedFeatures(img0,img1,matchedPoints1,matchedPoints2)
    
    S_data.t1.P=double(fliplr(matchedPoints2.Location))./HrScale;
    S_data.t0.P=double(fliplr(matchedPoints1.Location))./HrScale;
end