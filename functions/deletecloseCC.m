function S = deletecloseCC(S)
%this function checks how close keypoints are together, and discards them,
%in case they are too close

%load parameters, given in this script
global Suppression

keypoints = S.t1.CC;

%rescale 
keypoints = keypoints./Suppression;

%round to int
keypoints = round(keypoints);

% find 
[~, ind]= unique(keypoints, 'rows');
duplicate_ind = setdiff(1:size(keypoints, 1), ind);

S.t1.CC(duplicate_ind,:) = [];
S.t1.C(duplicate_ind,:) = [];
S.t1.F(duplicate_ind,:) = [];
S.t1.T(duplicate_ind,:) = [];


end

