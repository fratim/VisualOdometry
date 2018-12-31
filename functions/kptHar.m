function [S_data, running] = kptHar(S_data, img0, img1, boot)

    %load parameters
    global HrScale
    global HrKernel
    global MatchThresholdInit
    global HrQuality
    global BlockSize
    
    running = true;
    
    %resize images
    img0 = imresize(img0, HrScale);
    img1 = imresize(img1, HrScale);
    
    if(boot==1)
        points1 = detectHarrisFeatures(img0,'MinQuality',HrQuality,'FilterSize',HrKernel);
    end
    points2 = detectHarrisFeatures(img1,'MinQuality',HrQuality,'FilterSize',HrKernel);
    
    %Extract the neighborhood features.
    
    if(boot==1)
        [features1,valid_points1] = extractFeatures(img0,points1,'Method','Block','Blocksize',BlockSize);
    else
        [features1,valid_points1] = extractFeatures(img0,fliplr(S_data.t0.P)*HrScale,'Method','Block','Blocksize',BlockSize);
    end
    [features2,valid_points2] = extractFeatures(img1,points2,'Method','Block','Blocksize',BlockSize);
    
    %Match the features.

    indexPairs = matchFeatures(features1,features2,'MatchThreshold',MatchThresholdInit,'Unique',true);
    
    %Retrieve the locations of the corresponding points for each image.

    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    matchedPoints2 = valid_points2(indexPairs(:,2),:);
    
    %debug
    showMatchedFeatures(img0,img1,matchedPoints1,matchedPoints2)
    
    S_data.t1.P=double(fliplr(matchedPoints2.Location))./HrScale;
    
    if(boot==1)
        S_data.t0.P=double(fliplr(matchedPoints1.Location))./HrScale;
        S_data.ti.X = matchedPoints1.Location(:,1)./HrScale;
        S_data.ti.Y = matchedPoints1.Location(:,2)./HrScale;
        S_data.ti.X = [S_data.ti.X , matchedPoints2.Location(:,1)./HrScale];
        S_data.ti.Y = [S_data.ti.Y , matchedPoints2.Location(:,2)./HrScale];
    else
        S_data.ti.X = S_data.ti.X(indexPairs(:,1),:);
        S_data.ti.Y = S_data.ti.Y(indexPairs(:,1),:);
        S_data.ti.X = [S_data.ti.X , matchedPoints2.Location(:,1)./HrScale];
        S_data.ti.Y = [S_data.ti.Y , matchedPoints2.Location(:,2)./HrScale];
        S_data.t0.P=double(fliplr(matchedPoints1))./HrScale;
    end
    %showMatchedFeatures(img0,img1,fliplr(S_data.t0.P),fliplr(S_data.t1.P))
end