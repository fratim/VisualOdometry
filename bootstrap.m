function S =  bootstrap(img0, img1, K)%% Initialization of Pose and Landmarks

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
S.t1.P = zeros(1,1);
S.t1.X = zeros(1,1);
S.t1.C = zeros(1,1);
S.t1.F = zeros(1,1);
S.t1.T = zeros(1,1);
S.t1.Pose = zeros(3,4);

S.t0.P = zeros(1,1);
S.t0.X = zeros(1,1);
S.t0.C = zeros(1,1);
S.t0.F = zeros(1,1);
S.t0.T = zeros(1,1);
S.t0.Pose = eye(3,4);

%S(1:2,1:6) = {  P_t1,X_t1,C_t1,F_t1,T_t1, Pose_t1;...
%                P_t0,X_t0,C_t0,F_t0,T_t0, Pose_t0};

% 3.1 manually select two frames I_0 and I_1
% this is done in the Bootstrap part, when introducing the 'bootstap_frames' vector
    
% 3.2 Establish keypoint correspondences

% rescale factor for image processing (must be 1 for HARRIS)
rescale = 1;

% keypoint correspondences can be establisht either with HARRIS or SIFT
S = establishKptCorrespondencesHARRIS(S, img0, img1, rescale);

% 3.3 Relative pose estimation and triangulation of landmarks, use RANSAC 
S = estimaterelativePose_ML(S,K,1);
end