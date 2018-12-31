function [S, success] =  bootstrap(imgs, K)%% Initialization of Pose and Landmarks

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
S.ti.Y = [];
S.ti.X = [];
S.ti.WX = [];
S.ti.WY = [];
S.ti.WZ = [];

% 3.2 Establish keypoint correspondences
success = true;

% keypoint correspondences can be establisht either with HARRIS or SIFT
P_0=[];
%img0 = cell2mat(imgs(1));
%img1 = cell2mat(imgs(size(imgs,1)));
%[S, running, P_0] = kptHar(S, img0, img1, P_0,1);
for i=1:size(imgs,1)-1
    S.t0.P=S.t1.P;
    img0 = cell2mat(imgs(i));
    img1 = cell2mat(imgs(i+1));
    if(i==1)
        [S, running, P_0] = kptHar(S, img0, img1, P_0,1);
    else
        [S, running, P_0] = kptHar(S, img0, img1, P_0,0);
    end
end

S.t0.P = P_0;

if(~running)
    success = false;
    return
end

keep = ones(length(S.t1.P),1);
[S, running] = deletecloseFt(S, keep);

p_o = [S.ti.X(:,1) S.ti.Y(:,2)];
p_n = [S.ti.X(:,4) S.ti.Y(:,4)];
showMatchedFeatures(img0,img1,p_o,p_n)  
% 3.3 Relative pose estimation and triangulation of landmarks, use RANSAC 
%S.ti.P = S.t0.P;
%S.ti.X = S.t0.X;
[S, running] = estPose(S,K,1);

if(~running)
    success = false;
    return
end

end