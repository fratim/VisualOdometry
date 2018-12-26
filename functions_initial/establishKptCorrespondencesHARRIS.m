function S_data = establishKptCorrespondencesHARRIS(S_data, img0, img1, rescale)

    if (rescale ~= 1)
        disp('error, rescale for Harris is assumed to be 1!')
    end
    
    points1 = detectHarrisFeatures(img0,'MinQuality',0.01,'FilterSize',3);
    points2 = detectHarrisFeatures(img1,'MinQuality',0.01,'FilterSize',3);
    
    %Extract the neighborhood features.

    [features1,valid_points1] = extractFeatures(img0,points1);
    [features2,valid_points2] = extractFeatures(img1,points2);
    
    %Match the features.

    indexPairs = matchFeatures(features1,features2,'MatchThreshold',15,'Unique',true);
    
    %Retrieve the locations of the corresponding points for each image.

    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    matchedPoints2 = valid_points2(indexPairs(:,2),:);
    
    %debug
    %showMatchedFeatures(img0,img1,matchedPoints1,matchedPoints2)
    
    %have to be roundd, weird estimatefundamentalmatrix function
    S_data{1,1}=double(round(fliplr(matchedPoints2.Location)));
    S_data{2,1}=double(round(fliplr(matchedPoints1.Location)));
end