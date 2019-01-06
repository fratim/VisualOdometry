function S = contFt_KLT(S,img)
    
    %load parameters, given in this script
    global HrQuality
    global HrKernel
    global MinAngle
    global BlockSize
    global Suppression
    %needed for grid adding of keypoints
    [ymax,xmax] = size(img);
    
    points_new = detectHarrisFeatures(img,'MinQuality',HrQuality,'FilterSize',HrKernel);
    kpt_new = double(points_new.Location);

    % implement new keypoints and load old ones
    kpt_new_temp = round(kpt_new./Suppression);
    kpt_old_temp = round(S.t1.P./Suppression);

    % find duplicates of new corners
    exist_set = ismember(kpt_new_temp,kpt_old_temp,'rows');
    nonexist_set = find(exist_set==0);

    % discard new features that are already a landmark (already exist)
    kpt_new = points_new(nonexist_set,:);
    
    if(length(S.t1.CC>0))
        kpt_cand_temp = round(S.t1.CC./Suppression);
        % find duplicates of candidates
        exist_set = ismember(kpt_cand_temp,kpt_old_temp,'rows');
        nonexist_set = find(exist_set==0);

        % discard candidates that are already a landmark (already exist)
        S.t1.CC = S.t1.CC(nonexist_set,:);
        S.t1.C = S.t1.C(nonexist_set,:);
        S.t1.F = S.t1.F(nonexist_set,:);
        S.t1.T = S.t1.T(nonexist_set,:);
    end
    %extract features that belong to new keypoints (watch out for scale of image!!)
    %need feature extraction to determine uality of points
    [features_new,kpt_new_temp] = extractFeatures(img,kpt_new,'Method','Block','Blocksize',BlockSize);
    kpt_new = double(kpt_new_temp.Location);
    kpt_new_quality = kpt_new_temp.Metric;
    
    kpt_new_needed = 5;
    
    if(length(S.t1.P)<150)
        kpt_new_needed = 150;
    end
    
    [kpt_new,kpt_new_quality] = enforceBlocksKptNew(kpt_new,kpt_new_quality,kpt_new_needed, xmax, ymax);
    
    if(~isempty(S.t1.CC))
        
        % load old features
        %features_old = S.t1.F;
        
        %match features
        %indexPairs = matchFeatures(features_new,features_old,'MatchThreshold',MatchThresholdCont,'Unique',true);
 
        %find according image coordinates
        kpt_matched_new_xy = double(S.t1.CC);
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
            
            %load keypoints and other stuff
            p1 = kpt_matched_old_xy(u_temp,:);
            p2 = kpt_matched_new_xy(u_temp,:);
            kp_quality = S.t1.F;
            P1 = reshape(U(i,:),[3,4]);
            P2 = S.t1.Pose;

            R = P1(1:3,1:3)'*P2(1:3,1:3);
            T = (P2(1:3,4)-P1(1:3,4))';
            %Triangulate matched candidates
            
            blockSize = 50;
            
%             if(length(p1)>blockSize)
%                 p1_triang = [];
%                 p2_triang = [];
%                 X = [];
%                 npoints = length(p1);
%                 nblocks = floor(npoints/blockSize);
%                 for k = 1:nblocks
%                     if(blockSize*k>length(p1))
%                         p1_temp = p1(blockSize*k-blockSize+1:end,:);
%                         p2_temp = p1(blockSize*k-blockSize+1:end,:);
%                     else   
%                         p1_temp = p1(blockSize*k-blockSize+1:blockSize*k,:);
%                         p2_temp = p1(blockSize*k-blockSize+1:blockSize*k,:);
%                     end
%                     
%                     [p1_temp,p2_temp,X_temp] = triLnd(S,R,T,p1_temp,p2_temp,P1);
%                     p1_triang = [p1_triang;p1_temp];
%                     p2_triang = [p2_triang;p2_temp];
%                     X = [X;X_temp];
%                 end
%             else
            [p1_triang,p2_triang,X] = triLnd(S,R,T,p1,p2,P1);
%             end
            
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
                    newP = [newP;p2_triang(j,:)]; % flip to get u v
                    newX = [newX;X(j,:)];
                    newQuality = [newQuality; double(kp_quality(j))];
                end
            end
            %             
        end
        
        % add only 50 strongest keypoints in ech interation
        kptmax = 300;
        kptaddalways = 00;
        
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
 
        % Add new Features that were not matched with old ones
        S.t1.F = [S.t1.F;kpt_new_quality];
        S.t1.C = [S.t1.C;kpt_new];
        S.t1.CC = [S.t1.CC;kpt_new];
        T_add = repmat(reshape(S.t1.Pose,[1,12]),size(kpt_new,1),1);
        S.t1.T = [S.t1.T;T_add];
        
        disp(['Candidate points: ', num2str(length(S.t1.C))]);
        
        
    %Initial features
    else
        S.t1.F = [S.t1.F;double(kpt_new_quality)];
        S.t1.C = [S.t1.C;kpt_new];
        S.t1.CC = [S.t1.CC;kpt_new];
        T_add = repmat(reshape(S.t1.Pose,[1,12]),size(kpt_new,1),1);
        S.t1.T = [S.t1.T;T_add];
    end
   
end
