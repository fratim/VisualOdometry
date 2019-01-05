function S = contFt(S, img, K)
    
    %load parameters, given in this script
    global HrQuality
    global HrKernel
    global Suppression
    global MatchThresholdCont
    global MinAngle
    global BlockSize
    global kptadd
    
    points_new = detectHarrisFeatures(img,'MinQuality',HrQuality,'FilterSize',HrKernel);
    
    kpt_new = double(points_new.Location);

    % implement new keypoints and load old ones
    kpt_new_temp = round(kpt_new);
    kpt_old_temp = round(S.t1.P);

    % find duplicates
    exist_set = ismembertol(kpt_new_temp,kpt_old_temp,'ByRows',2);
    nonexist_set = find(exist_set==0);

    % discard keypoints that are already a landmark (already exist)
    kpt_new = points_new(nonexist_set,:);
    
    %extract features that belong to new keypoints (watch out for scale of image!!)
    [features_new,kpt_new_temp] = extractFeatures(img,kpt_new,'Method','Block','Blocksize',BlockSize);
    kpt_new = kpt_new_temp.Location;
    kpt_new_quality = kpt_new_temp.Metric;
    
    if(~isempty(S.t1.F))
        
        % load old features
        features_old = S.t1.F;
        
        %match features
        indexPairs = matchFeatures(features_new,features_old,'MatchThreshold',MatchThresholdCont,'Unique',true);
 
        %find according image coordinates
        kpt_matched_new_xy = double(kpt_new);
        kpt_matched_old_xy = double(S.t1.C);
        
        % plot on new picture 
        % showMatchedFeatures(img,img,kpt_matched_old_xy,kpt_matched_new_xy) 
        
        %Find different feature starting points
        U = unique(S.t1.T,'rows');
        
        %new features and landmarks to be added to S.t1.P and S.t1.X
        newP = [];
        newX = [];
        newQuality = [];
        % start i at 3, so at least 2 pictures difference
        %Check the angle criterium
        for i=1:size(U,1)
            %Cluster different feature starting points 
            u_temp = ismember(S.t1.T,U(i,:),'rows');
            u_temp = find(u_temp==1); 

            %Get intersection of clustered and matched points
            [C,i_k,i_u] = intersect(indexPairs(:,2),u_temp);

            %Get indices for old and new points
            idx_o = indexPairs(i_k,2);
            idx_n = indexPairs(i_k,1);

            p1 = kpt_matched_old_xy(idx_o,:);
            p2 = kpt_matched_new_xy(idx_n,:);
            kp_quality = kpt_new_quality(idx_n);
            P1 = reshape(U(i,:),[3,4]);
            P2 = S.t1.Pose;

            R = P1(1:3,1:3)'*P2(1:3,1:3);
            T = (P2(1:3,4)-P1(1:3,4))';
            %Triangulate matched candidates
            [p1,p2,X] = triLnd(S,R,T,p1,p2,P1);

            if(isempty(X))
                continue
            end
                     
            a = norm(P1(1:3,4)-P2(1:3,4));

            for j=1:size(X,1)
                %Cosine rule                
                b = norm(P1(1:3,4)-X(j,:));
                c = norm(P2(1:3,4)-X(j,:));
                alpha = acos((a*a -b*b -c*c)/(-2*b*c));

                %Add features that fulfill criterium
                if(abs(alpha)>MinAngle)
                    newP = [newP;p2(j,:)]; % flip to get u v
                    newX = [newX;X(j,:)];
                    newQuality = [newQuality; double(kp_quality(j))];
                end
            end
            %             
        end
        
        %needed for grid adding of keypoints
        [ymax,xmax] = size(img);
        
        % add only 50 strongest keypoints in ech interation
        kptmax = 1000;
        kptaddalways = 500;
        
        if(length(newP)<=kptaddalways)
            %just add all keypoints avaliable
            newP = newP;
            newX = newX;
        elseif (length(S.t1.P)<kptmax)
            % if less than max, only fill up + addalways
            kpt_needed = kptmax - length(S.t1.P) + kptaddalways;
            % if more newP than the ones added, choose the best ones with
            % grid
            if(length(newP)>kpt_needed)
                [newP,newX] = enforceBlocksKpt(newP, newX, newQuality, kpt_needed, xmax, ymax);
            end
        else
            % if full, jsut add the addalways keypoints
            kpt_needed = kptaddalways;
            [newP,newX] = enforceBlocksKpt(newP, newX, newQuality, kpt_needed, xmax, ymax);
        end
            
        S.t1.P = [S.t1.P; newP];
        S.t1.X = [S.t1.X; newX];
        
        % Remove lost features (take only the matched ones)
        S.t1.F = S.t1.F(indexPairs(:,2),:);
        S.t1.C = S.t1.C(indexPairs(:,2),:);
        S.t1.T = S.t1.T(indexPairs(:,2),:);
 
        % Add new Features that were not matched with old ones
        new_feat_ind = setdiff(1:size(kpt_new,1),indexPairs(:,1));
        S.t1.F = [S.t1.F;features_new(new_feat_ind,:)];
        S.t1.C = [S.t1.C;kpt_new(new_feat_ind,:)];
        T_add = repmat(reshape(S.t1.Pose,[1,12]),size(new_feat_ind,2),1);
        S.t1.T = [S.t1.T;T_add];
        
        
    %Initial features
    else
        S.t1.F = [S.t1.F;features_new];
        S.t1.C = [S.t1.C;kpt_new];
        T_add = repmat(reshape(S.t1.Pose,[1,12]),size(kpt_new,1),1);
        S.t1.T = [S.t1.T;T_add];
    end
   
end
