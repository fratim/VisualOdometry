function S = contFt_KLT(S,img)
    
    %load parameters, given in this script
    global HrQuality
    global HrKernel
    global MinAngle
    global BlockSize
    global Suppression
    global PGoal
    global kptadd
    global PGoal
    global n_crit
    global kptmax
    global kptaddalways
    %needed for grid adding of keypoints
    [ymax,xmax] = size(img);
    
    Harris_new = detectHarrisFeatures(img,'MinQuality',HrQuality,'FilterSize',HrKernel);

    % implement new keypoints and load old ones
    %kpt_new_temp = round(kpt_new./Suppression);
    %kpt_old_temp = round(S.t1.P./Suppression);

    %find duplicates of new corners (already a landmark)
    exist_set = ismembertol(Harris_new.Location,S.t1.P,Suppression,'ByRows',true,'DataScale',1);
    nonexist_set = find(exist_set==0);
    Harris_new = Harris_new(nonexist_set,:);
    
    if(~isempty(S.t1.CC))
        %dicard features that are already a Candidate
        exist_set = ismembertol(Harris_new.Location,S.t1.CC,Suppression,'ByRows',true,'DataScale',1);
        nonexist_set = find(exist_set==0);
        Harris_new = Harris_new(nonexist_set,:);
    end
    
    % add new keypoints
    Harris_new_needed = 50;
    
    if(length(S.t1.P)<PGoal)
        Harris_new_needed = 150;
    end
    
    %if(length(S.t1.CC)<250)
    %    Harris_new_needed = 150;
    %end
    
    %extract features that belong to new keypoints (watch out for scale of image!!)
    %need feature extraction to determine uality of points
    [features_new,kpt_new_temp] = extractFeatures(img,Harris_new,'Method','Block','Blocksize',BlockSize);
    Harris_new = double(kpt_new_temp.Location);
    kpt_new_quality = kpt_new_temp.Metric;
    
    [Harris_new,kpt_new_quality] = deletecloseHarris(Harris_new,kpt_new_quality);
    
    [Harris_new,kpt_new_quality] = enforceBlocksKptNew(Harris_new,kpt_new_quality,Harris_new_needed, xmax, ymax);
    
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
        
         if(length(S.t1.P)<n_crit)
             if(size(U,1)>2)
                 a=2;
             else
                 a=1;
            end
            critical=1;
        else 
            a=1;
             critical = 0;
        end
      
        for i=a:size(U,1)
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
                if(abs(alpha)>MinAngle || critical==1)
                    newP = [newP;p2(j,:)]; % flip to get u v
                    newX = [newX;X(j,:)];
                    newQuality = [newQuality; double(kp_quality(j))];
                end
            end
                        
        end
        
        % add only 50 strongest keypoints in ech interation        
        if(length(newP)<=kptaddalways)
            %just add all keypoints avaliable
            %newP = newP;
            %newX = newX;
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
        S.t1.C = [S.t1.C;Harris_new];
        S.t1.CC = [S.t1.CC;Harris_new];
        T_add = repmat(reshape(S.t1.Pose,[1,12]),size(Harris_new,1),1);
        S.t1.T = [S.t1.T;T_add];
        
        
        % find duplicates of candidates
        exist_set = ismembertol(S.t1.CC,S.t1.P,Suppression,'ByRows',true,'DataScale',1);
        nonexist_set = find(exist_set==0);

        S.t1.CC = S.t1.CC(nonexist_set,:);
        S.t1.C = S.t1.C(nonexist_set,:);
        S.t1.F = S.t1.F(nonexist_set,:);
        S.t1.T = S.t1.T(nonexist_set,:);

        len_old = length(S.t1.CC);
            
        S = deletecloseCC(S);
        
        disp(['points CC deleted supression: ', num2str(len_old-length(S.t1.CC))]);
        
        disp(['Candidate points: ', num2str(length(S.t1.C))]);
        
        
        
    %Initial features
    else
        S.t1.F = [S.t1.F;double(kpt_new_quality)];
        S.t1.C = [S.t1.C;Harris_new];
        S.t1.CC = [S.t1.CC;Harris_new];
        T_add = repmat(reshape(S.t1.Pose,[1,12]),size(Harris_new,1),1);
        S.t1.T = [S.t1.T;T_add];
    end
   
end
