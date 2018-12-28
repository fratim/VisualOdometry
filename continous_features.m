function S_data = continous_features(S_data, img, rescale, K)

    if (rescale ~= 1)
        disp('error, rescale for Harris is assumed to be 1!')
    end
    
    points_new = detectHarrisFeatures(img,'MinQuality',0.01,'FilterSize',3);
    
    %Extract the neighborhood features.

    [features,valid_points] = extractFeatures(img,points_new);
    
    if(~isempty(S_data.t1.F))
        %Match the features.
        features_old = binaryFeatures(S_data.t1.F);
        indexPairs = matchFeatures(features,features_old,'MatchThreshold',15,'Unique',true);

        %Retrieve the locations of the corresponding points for each image.
        matchedPoints_new = valid_points(indexPairs(:,1),:);
        matchedPoints_new = double(round(fliplr(matchedPoints_new.Location)));
        matchedPoints_old = S_data.t1.C(indexPairs(:,2),:);

        %Check the angle criterium
        for i=1:size(indexPairs(:,1),1)
            p1 = matchedPoints_old(i,:);
            p2 = matchedPoints_new(i,:);
            P1 = reshape(S_data.t1.T(i,:),[3,4]);
            P2 = S_data.t1.Pose;
            X = triangulateNewLandmarklinear(p1,p2,P1,P2,K);
            %Cosine rule
            a = norm(P1(1:3,4)-P2(1:3,4));
            b = norm(P1(1:3,4)-X);
            c = norm(P2(1:3,4)-X);
            alpha = acos((a*a -b*b -c*c)/(-2*b*c));
            
            %Add features that fulfill criterium
            if(abs(alpha)>3/180 * pi)
                S_data.t1.P = [S_data.t1.P;p2];
                S_data.t1.X = [S_data.t1.X;X];
            end
            
        end


        %Remove lost features
        S_data.t1.F = S_data.t1.F(indexPairs(:,2),:);
        S_data.t1.C = S_data.t1.C(indexPairs(:,2),:);
        S_data.t1.T = S_data.t1.T(indexPairs(:,2),:);

        %Add new Features that were not matched with old ones
        new_feat_ind = setdiff(1:size(valid_points.Location,1),indexPairs(:,1));
        S_data.t1.F = [S_data.t1.F;features.Features(new_feat_ind,:)];
        S_data.t1.C = [S_data.t1.C;valid_points.Location(new_feat_ind,:)];
        T_add = repmat(reshape(S_data.t1.Pose,[1,12]),size(new_feat_ind,2),1);
        S_data.t1.T = [S_data.t1.T;T_add];
      
    %Initial features
    else
        S_data.t1.F = [S_data.t1.F;features.Features];
        S_data.t1.C = [S_data.t1.C;valid_points.Location()];
        T_add = repmat(reshape(S_data.t1.Pose,[1,12]),size(valid_points.Location,1),1);
        S_data.t1.T = [S_data.t1.T;T_add];
    end
    
    
    %debug
    %showMatchedFeatures(img0,img1,matchedPoints1,matchedPoints2)
    
    %have to be roundd, weird estimatefundamentalmatrix function
    %S_data.t1.P=double(round(fliplr(matchedPoints2.Location)));
    %S_data.t0.P=double(round(fliplr(matchedPoints1.Location)));
end