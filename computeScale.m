function [S, median_scale] = computeScale(S,R,T,freq_idx,inliersIndex)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%load old landmarks (reference scale)

        old_landmarks = S.t1.X;
        
        % triangulate new landmarks using keypoints of current fram and
        % frame -3
        
        [S, running] = triLndCont(S,R,T,freq_idx(inliersIndex));
        new_landmarks = S.t1.X;
        
        old_nonnan = find(~isnan(old_landmarks(:,1)));
        new_nonnan = find(~isnan(new_landmarks(:,1)));
        
        both_nonnan = intersect(old_nonnan,new_nonnan);
        
        old_landmarks = old_landmarks(both_nonnan,:);
        new_landmarks = new_landmarks(both_nonnan,:);
        
        rand_idx = randperm(length(both_nonnan));
        
        landmarks_old_rand = old_landmarks(rand_idx,:);
        landmarks_new_rand = new_landmarks(rand_idx,:);
        diff_old = diff(landmarks_old_rand);
        diff_new = diff(landmarks_new_rand);
        distances_old = sqrt(sum(diff_old.^2,2));
        distances_new = sqrt(sum(diff_new.^2,2));
        scales = distances_old./distances_new;
        median_scale = median(scales);
        disp(median_scale)
        T_new = [S.t0.Pose;zeros(1,3),1]*[R,median_scale*T;zeros(1,3),1];
        S.t1.Pose = T_new(1:3,1:4);
        
        % rescale landmarks (multiplication)
        new_landmarks = new_landmarks*median_scale;
        
        %write into struct
        S.t1.X = NaN*ones(length(S.ti.X),3);
        S.t1.X(both_nonnan,:) = new_landmarks;
end

