function [S, running] = deletecloseFt(S, keep)
%this function checks how close keypoints are together, and discards them,
%in case they are too close

%load parameters, given in this script
global Suppression
global MinPoints

running = true;

keypoints = [S.ti.X(:,4),S.ti.Y(:,4)];

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
S.ti.X = S.ti.X(idx,:);
S.ti.Y = S.ti.Y(idx,:);

if(~isempty(S.t0.X)) 
    %S.t0.X = S.t0.X(find(keep>0),:);
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

