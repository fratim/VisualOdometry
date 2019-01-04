function [S, running] = estPose(S,isBoot)
    
    global NumTrials
    global DistanceThreshold
    global InlierPercentage
    global Confidence
    global MaxReprojErrorCameraPose

    running = true;
    
    if(isBoot)
        %If we boot, triangulate first landmarks
        [S.t0.P,S.t1.P,S.t1.X,running]=landmarks(S,S.t0.P,S.t1.P,S.t0.Pose);
      
    else
        %Calculate current pose from world landmarks and correspondences
        [R,T_t,inliersIdx] = estimateWorldCameraPose(...
            S.t1.P,S.t1.X,S.K,'MaxReprojectionError',...
            MaxReprojErrorCameraPose,'MaxNumTrials',NumTrials);
        
        inlierpercentage = length(find(inliersIdx>0))/length(inliersIdx);
        disp(['inliers in estFund of estWorldCamera: ', num2str(inlierpercentage)]);
        
        inliers = find(inliersIdx>0);
        outliers = find(inliersIdx==0);
        
        keep = 0.65;
        
        inliers = [inliers; outliers(1:round(length(outliers)*keep))];
        
        %Reject outliers
        S.t1.P=S.t1.P(inliers,:);
        S.t0.P=S.t0.P(inliers,:);
        S.t1.X=S.t1.X(inliers,:);
        S.t1.Pose=[R,T_t'];
        
        disp(['points after estimateWorldCameraPose: ',num2str(length(S.t1.P))]);
    end
    

%     
%     if (isBoot==0)
%         %recover scale factor
% %         rand_idx = randperm(length(S.t0.X));
% %         landmarks_old = S.t0.X(rand_idx,:);
% %         landmarks_new = S.t1.X(rand_idx,:);
% %         diff_old = diff(landmarks_old);
% %         diff_new = diff(landmarks_new);
% %         distances_old = sqrt(sum(diff_old.^2,2));
% %         distances_new = sqrt(sum(diff_new.^2,2));
% %         scales = distances_old./distances_new;
% %         median_scale = median(scales);
%         median_scale = 1;
%         
%         T_new = [S.t0.Pose;zeros(1,3),1]*[R,median_scale*T;zeros(1,3),1];
%         %S.t1.Pose = T_new(1:3,1:4);
%         
%         % triangulate landmarks with correct camera position 
%         
%         [S, running] = triLndCont(S,K,R,median_scale*T,freq_idx(inliersIndex),isBoot);
        
       
    
    
    if(~running)
        return
    end
    
     % print messages
    disp('Current estimated Pose')
    disp(S.t1.Pose)
    
end