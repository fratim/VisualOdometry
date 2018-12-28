function S_data = continous_features(S_data, img, K)
    
    %load parameters, given in this script
    run ParkingParameters.m
    
    img = imresize(img, HrScale);
    points_new = detectHarrisFeatures(img,'MinQuality',HrQuality,'FilterSize',HrKernel);
    
    kpt_new_xy = double(points_new.Location/HrScale);
    
    downsample = 2;
    
    %Extract the neighborhood features.
 
    %[features,valid_points] = extractFeatures(img,points_new);
    
    %rescale image ccordinates
    %valid_points.Location = valid_points.Location./HrScale;

    % implement new keypoints and load old ones
    kpt_new_temp_xy = round(kpt_new_xy/downsample);
    kpt_old_temp_xy = round(fliplr(S_data.t1.P)./2);

    % find duplicates
    exist_set = ismembertol(kpt_new_temp_xy,kpt_old_temp_xy,'ByRows',2);
    nonexist_set = find(exist_set==0);

    % discard keypoints that are already a landmark (already exist)
    kpt_new_xy = kpt_new_xy(nonexist_set,:);

    %extract features that belong to new keypoints (watch out for scale of image!!)
    [features_new,kpt_new_temp] = extractFeatures(img,kpt_new_xy*HrScale);
    kpt_new_xy = kpt_new_temp/HrScale;
    
    if(~isempty(S_data.t1.F))
        
        % load old features
        features_old = S_data.t1.F;
        
        %match features
        indexPairs = matchFeatures(features_new,features_old,'MatchThreshold',15,'Unique',true);
 
        %find according image coordinates
        kpt_matched_new_xy = double(kpt_new_xy(indexPairs(:,1),:));
        kpt_matched_old_xy = S_data.t1.C(indexPairs(:,2),:);
        
%         %Check the angle criterium
       for i=1:size(indexPairs(:,1),1)
            p1 = double(kpt_matched_old_xy(i,:));
            p2 = kpt_matched_new_xy(i,:);
            P1 = reshape(S_data.t1.T(i,:),[3,4]);
            P2 = S_data.t1.Pose;
            X = triangulateNewLandmarklinear(p1,p2,P1,P2,K);
            
            if(isempty(X))
                continue
            end
%             
%             %Cosine rule
%             a = norm(P1(1:3,4)-P2(1:3,4));
%             b = norm(P1(1:3,4)-X);
%             c = norm(P2(1:3,4)-X);
%             alpha = acos((a*a -b*b -c*c)/(-2*b*c));
%             
%             %Add features that fulfill criterium
%             %if(abs(alpha)>3/180 * pi)
                S_data.t1.P = [S_data.t1.P;fliplr(p2)]; % flip to get u v
                S_data.t1.X = [S_data.t1.X;X];
%             %end
%             
        end
        
        % Remove lost features (take only the matched ones)
        S_data.t1.F = S_data.t1.F(indexPairs(:,2),:);
        S_data.t1.C = S_data.t1.C(indexPairs(:,2),:);
        S_data.t1.T = S_data.t1.T(indexPairs(:,2),:);
 
        % Add new Features that were not matched with old ones
        new_feat_ind = setdiff(1:size(kpt_new_xy,1),indexPairs(:,1));
        S_data.t1.F = [S_data.t1.F;features_new(new_feat_ind,:)];
        S_data.t1.C = [S_data.t1.C;kpt_new_xy(new_feat_ind,:)];
        T_add = repmat(reshape(S_data.t1.Pose,[1,12]),size(new_feat_ind,2),1);
        S_data.t1.T = [S_data.t1.T;T_add];
      
    %Initial features
    else
        S_data.t1.F = [S_data.t1.F;features_new];
        S_data.t1.C = [S_data.t1.C;kpt_new_xy];
        T_add = repmat(reshape(S_data.t1.Pose,[1,12]),size(kpt_new_xy,1),1);
        S_data.t1.T = [S_data.t1.T;T_add];
    end
   
end
