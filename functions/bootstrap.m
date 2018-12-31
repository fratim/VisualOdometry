function [S, success] =  bootstrap(img0, img1, K)%% Initialization of Pose and Landmarks

% establish S data struct (cell)
% Markovian idea: cell of information (denoted by S) is passed and updated continously,
% computation of S(t+1) solely depends on S(t)
% Cell struct S should contain: Keypoints (P), associated landmarks (X),
% set of candidate keypoints (C), candidate first observation (F),
% candidate first camera pose (T)
% 
% For the beginning, the VO pipeline can work with only the keypoints given by the initialization,
% afterwards the method of triangulating new landmarks can be implemnted
% (this requires C, F and T)

% Second row can save values at prior timestep (indicated by _t0), while
% first row saves values at current timestep (indicated by _t1) this is
% needed for the computation of the relative scale factor

% Therfore, the data struct does depend on two timesteps, the current and
% the previous

% Additionally, it holds the current pose, with repspect to the very
% first frame. This pose is saved as [R, T] (3x4)

%S = cell(2,6);
%S = struct('P_t1',{},'X_t1',{},'C_t1',{})
S.t1.P = [];
S.t1.X = [];
S.t1.C = [];
S.t1.F = [];
S.t1.T = [];
S.t1.Pose = [];

S.t0.P = [];
S.t0.X = [];
S.t0.C = [];
S.t0.F = [];
S.t0.T = [];
S.t0.Pose = eye(3,4);
S.K = cameraParameters('IntrinsicMatrix',K');

% 3.2 Establish keypoint correspondences
success = true;

% keypoint correspondences can be establisht either with HARRIS or SIFT
[S, running] = trial_kptHar(S, img0, img1);

if(~running)
    success = false;
    return
end

keep = ones(length(S.t1.P),1);
[S, running] = deletecloseFt(S, keep);

% 3.3 Relative pose estimation and triangulation of landmarks, use RANSAC
[height, width] = size(img0);
[S, running] = estPose(S,K,1,width,height);

if(~running)
    success = false;
    return
end

end