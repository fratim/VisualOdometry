function S_data = contFt(S_data, img, K)
    
    %load parameters, given in this script
    global HrQuality
    global HrKernel
    global HrScale
    global Suppression
    global MatchThresholdCont
    global MinAngle
    global BlockSize
    
    p_oo= [S_data.ti.X(:,4),S_data.ti.Y(:,4)];
    img = imresize(img, HrScale);
    points_new = detectHarrisFeatures(img,'MinQuality',HrQuality,'FilterSize',HrKernel);
    points_new = selectStrongest(points_new,1000);
    kpt_new_xy = double(points_new.Location/HrScale);

    % implement new keypoints and load old ones
    kpt_new_temp_xy = round(kpt_new_xy/Suppression);
    kpt_old_temp_xy = round(p_oo./Suppression);

    % find duplicates
    exist_set = ismembertol(kpt_new_temp_xy,kpt_old_temp_xy,'ByRows',2);
    nonexist_set = find(exist_set==0);

    % discard keypoints that are already a landmark (already exist)
    kpt_new_xy = kpt_new_xy(nonexist_set,:);

    %extract features that belong to new keypoints (watch out for scale of image!!)
    [features_new,kpt_new_temp] = extractFeatures(img,kpt_new_xy*HrScale,'Method','Block','Blocksize',BlockSize);
    kpt_new_xy = kpt_new_temp/HrScale;
    
    if(~isempty(S_data.t1.F))
        
        % load old features
        features_old = S_data.t1.F;
        
        %match features
        indexPairs = matchFeatures(features_new,features_old,'MatchThreshold',MatchThresholdCont,'Unique',true);
 
        %find according image coordinates
        kpt_matched_new_xy = double(kpt_new_xy);
        kpt_matched_old_xy = double(S_data.t1.C);
        
        %Find different feature starting points
        U = unique(S_data.t1.T,'rows');

        ldm_added = 0;
        %Check the angle criterium
        for i=1:size(U,1)
            %Cluster different feature starting points 
            u_temp = ismember(S_data.t1.T,U(i,:),'rows');
            u_temp = find(u_temp==1); 

            %Get intersection of clustered and matched points
            [C,i_k,i_u] = intersect(indexPairs(:,2),u_temp);

            %Get indices for old and new points
            idx_o = indexPairs(i_k,2);
            idx_n = indexPairs(i_k,1);

            p1 = kpt_matched_old_xy(idx_o,:);
            p2 = kpt_matched_new_xy(idx_n,:);
            P1 = reshape(U(i,:),[3,4]);
            P2 = S_data.t1.Pose;

            %Triangulate matched candidates
            X = triLndNew(p1,p2,P1,P2,S_data.K);

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
                    S_data.t1.P = [S_data.t1.P;fliplr(p2(j,:))]; % flip to get u v
                    S_data.t1.X = [S_data.t1.X;X(j,:)];
                    ldm_added = ldm_added +1;
                    S_data.ti.X = [S_data.ti.X;[NaN NaN NaN p2(j,1)]];
                    S_data.ti.Y = [S_data.ti.Y;[NaN NaN NaN p2(j,2)]];
                end
            end
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
        
        disp([  'features tracked: ', num2str(length(S_data.t0.X)),' //landmarks added: ',...
                num2str(ldm_added), ' //candidates added: ', num2str(length(new_feat_ind(new_feat_ind>0)))])
        
        
    %Initial features
    else
        S_data.t1.F = [S_data.t1.F;features_new];
        S_data.t1.C = [S_data.t1.C;kpt_new_xy];
        T_add = repmat(reshape(S_data.t1.Pose,[1,12]),size(kpt_new_xy,1),1);
        S_data.t1.T = [S_data.t1.T;T_add];
    end
   
end
