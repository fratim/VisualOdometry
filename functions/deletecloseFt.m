function [S, keep] = deletecloseFt(S, keep)
%this function checks how close keypoints are together, and discards them,
%in case they are too close

%load parameters, given in this script
global Suppression

keypoints = S.t1.P;

%rescale 
keypoints = keypoints./Suppression;

%round to int
keypoints = round(keypoints);

% find 
[~, ind]= unique(keypoints, 'rows');
duplicate_ind = setdiff(1:size(keypoints, 1), ind);
keep(duplicate_ind)=0;

%debug
%disp("points deleted that were too close: ")
%disp(length(duplicate_ind))

end

