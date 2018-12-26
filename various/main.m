clear all
close all
clc

%% Setup
ds = 2; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    parking_path = './parking'
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
if ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    bootstrap_frames = [0,8];
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end


%% Initialization of Pose and Landmarks

% establish S data struct (cell), explained later (4.1)
% second row can save values at prior timestep (indicated by _t0), while
% first row saves values at current timestep (indicated by _t1) this is only needed for the
% initialization and possibly further debugging

% additionally, it could hold the current pose, with repspect to the very
% first frame. This pose is saved as [R, T] (3x4)
S = cell(2,6);

P_t0 = zeros(1,1);
X_t0 = zeros(1,1);
C_t0 = zeros(1,1);
F_t0 = zeros(1,1);
T_t0 = zeros(1,1);
Pose_t0 = eye(3,4);

P_t1 = zeros(1,1);
X_t1 = zeros(1,1);
C_t1 = zeros(1,1);
F_t1 = zeros(1,1);
T_t1 = zeros(1,1);
Pose_t1 = zeros(3,4);

S(1:2,1:6) = {  P_t1,X_t1,C_t1,F_t1,T_t1, Pose_t1;...
                P_t0,X_t0,C_t0,F_t0,T_t0, Pose_t0};

% 3.1 manually select two frames I_0 and I_1
    % this is done in the Bootstrap part, when introducing the 'bootstap_frames' vector
% 3.2 Establish keypoint correspondences (using KLT or patch matching)

    S = establishKptCorrespondencesSIFT(S, img0, img1);

% 3.3 Relative pose estimation and triangulation of landmarks, use RANSAC 

    S = estimaterelativePoseSIFT(S, K);
    
    S = triangulateLandmarkslinear(S, K); 

% 3.4 Initialize VO pipeline with inlier keypoints and associated landmarks
    % This function must return P_0 (Keypoints) and X_0 (associated landmarks)
    
    % possible simply cut of the second row of the S cell
    % S = S(1,:);

%% Continous VO Pipeline (must be called in continous operation bodey)

% 4.1 Markovian idea: cell of information (denoted by S) is passed and updated continously,
% computation of S(t+1) solely depends on S(t)
% Cell struct S should contain: Keypoints (P), associated landmarks (X),
% set of candidate keypoints (C), candidate first observation (F),
% candidate first camera pose (T)
% 
% For the beginning, the VO pipeline can work with only the keypoints given by the initialization,
% afterwards the method of triangulating new landmarks can be mplemnted
% (this requires C, F and T)


%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    % Makes sure that plots refresh.    
    pause(0.01);
    
    prev_img = image;
    
    imshow(image)
    waitforbuttonpress
end
