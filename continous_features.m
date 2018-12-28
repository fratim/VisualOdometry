function S_data = continous_features(S_data, img, K)
    
    %load parameters, given in this script
    run ParkingParameters.m
    
    points_new = detectHarrisFeatures(img,'MinQuality',HrQuality,'FilterSize',HrKernel);
    
    img = imresize(img, HrScale);
    %Extract the neighborhood features.

    [features,valid_points] = extractFeatures(img,points_new);
    
    %rescale image ccordinates
    valid_points.Location = valid_points.Location./HrScale;

    if(~isempty(S_data.t1.F))

        a = round(valid_points.Location./2);
        b = round(fliplr(S_data.t1.P)./2);
        used_set = ismembertol(a,b,'ByRows',2);
        unused_set = find(used_set==0);
        features = features.Features(unused_set,:);
        valid_points = valid_points.Location(unused_set,:);
        
        features = binaryFeatures(features);
        features_old = binaryFeatures(S_data.t1.F);
        indexPairs = matchFeatures(features,features_old,'MatchThreshold',15,'Unique',true);

        %Retrieve the locations of the corresponding points for each image.
        matchedPoints_new = valid_points(indexPairs(:,1),:);
        matchedPoints_new = double(fliplr(matchedPoints_new));
        matchedPoints_old = S_data.t1.C(indexPairs(:,2),:);
        
        [~, ind]= unique(S_data.t1.P, 'rows');
        if(size(ind,1)~=size(S_data.t1.P,1))
            disp('HOOOOSSSSAA222')
        end
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
        [~, ind]= unique(S_data.t1.P, 'rows');
        if(size(ind,1)~=size(S_data.t1.P,1))
            disp('HOOOOSSSSAA333')
        end

        %Remove lost features
        S_data.t1.F = S_data.t1.F(indexPairs(:,2),:);
        S_data.t1.C = S_data.t1.C(indexPairs(:,2),:);
        S_data.t1.T = S_data.t1.T(indexPairs(:,2),:);

        %Add new Features that were not matched with old ones
        new_feat_ind = setdiff(1:size(valid_points,1),indexPairs(:,1));
        S_data.t1.F = [S_data.t1.F;features.Features(new_feat_ind,:)];
        S_data.t1.C = [S_data.t1.C;valid_points(new_feat_ind,:)];
        T_add = repmat(reshape(S_data.t1.Pose,[1,12]),size(new_feat_ind,2),1);
        S_data.t1.T = [S_data.t1.T;T_add];
        
        [~, ind]= unique(S_data.t1.P, 'rows');
        if(size(ind,1)~=size(S_data.t1.P,1))
            disp('HOOOOSSSSAA3333')
        end
      
    %Initial features
    else
        S_data.t1.F = [S_data.t1.F;features.Features];
        S_data.t1.C = [S_data.t1.C;valid_points.Location()];
        T_add = repmat(reshape(S_data.t1.Pose,[1,12]),size(valid_points.Location,1),1);
        S_data.t1.T = [S_data.t1.T;T_add];
    end
   
end