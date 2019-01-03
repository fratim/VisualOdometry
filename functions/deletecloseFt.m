function [S, running] = deletecloseFt(S, keep)
%this function checks how close keypoints are together, and discards them,
%in case they are too close

%load parameters, given in this script
global Suppression
global MinPoints

running = true;

keypoints = S.t1.P;

%rescale 
keypoints = keypoints./Suppression;

%round to int
keypoints = round(keypoints);

% find 
[~, ind]= unique(keypoints, 'rows');
duplicate_ind = setdiff(1:size(keypoints, 1), ind);
keep(duplicate_ind)=0;

% delete keypoints and landmarkes that are discarded
idx = find(keep>0);
S.t1.P = S.t1.P(idx,:);
S.t0.P = S.t0.P(idx,:);

if(~isempty(S.t1.X)) 
    %S.t0.X = S.t0.X(find(keep>0),:);
    S.t1.X = S.t1.X(idx,:);
    S.t0.X = S.t0.X(idx,:);
end
% break here if less than Minpoints keypoints are tracked
if length(keep(keep>0)) < MinPoints
    disp('Less than Minpoints keypoints tracked, execution stopped')
    running = false;
end

%debug
%disp("points deleted that were too close: ")
%disp(length(duplicate_ind))

end

