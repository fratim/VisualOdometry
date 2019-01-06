function [Harris_new,kpt_new_quality] = deletecloseHarris(Harris_new,kpt_new_quality)
%this function checks how close keypoints are together, and discards them,
%in case they are too close

%load parameters, given in this script
global Suppression

keypoints = Harris_new;

%rescale 
keypoints = keypoints./Suppression;

%round to int
keypoints = round(keypoints);

% find 
[~, ind]= unique(keypoints, 'rows');
duplicate_ind = setdiff(1:size(keypoints, 1), ind);

Harris_new(duplicate_ind,:) = [];
kpt_new_quality(duplicate_ind,:) = [];

end

