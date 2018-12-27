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
    parking_path = './parking';
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
    bootstrap_frames = [0,4];
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

%% Initialization of Pose and Landmarks

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
S = estimaterelativePose(S, K);

% optional: fit first Pose to initial goundtruth pos, to recover unknown
% scale factor
% S{1,6}(1:3,4)=S{1,6}(1:3,4)*ground_truth(bootstrap_frames(2)+1,1)
    
% Debug plot results
debug = true;
if (debug == true)
    debugplot(S, img0, img1)
end

disp('Position estimate after initialization:')
disp(S.t1.Pose)

%% Continuous operation

close all
range = (bootstrap_frames(2)+1):last_frame;
% possible only take every xth frame (right now every second)
%range = range(1):2:range(end);

%keypoints = flipud(S.t0.P');
r_T = 15;
bs = 2*r_T+1;
num_iters = 50;
%dkp = zeros(2,size(S.t0.P,1));
%keep = zeros(1,size(S.t0.P,1));
lambda = 3;

pointTracker = vision.PointTracker('NumPyramidLevels',3, ...
        'MaxBidirectionalError',lambda,'BlockSize',[bs,bs],'MaxIterations',num_iters);
%initialize(pointTracker,S.t1.P,img1);
image = img1;
for i = range
    %S.t1.P = flipud(S.t1.P);
    keypoints = flipud(S.t1.P);
    initialize(pointTracker,keypoints,image);
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
    pause(5);
    
    % if first interation, use last bootstrap picture
    if (i == range(1))
        %load last picture used for bootstrap
        prev_image = im2uint8(rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))])));
    end
    
    % new timestep, therefore update S
    S.t0 = S.t1;
    
    %S(1,1:6) = {P_t1,X_t1,C_t1,F_t1,T_t1, Pose_t1};
    
    % flip to meet x y convention

    
    %for j = 1:size(S.t0.P,1)
    %    [W, keep(j)] = trackKLTRobustly(prev_image, image, keypoints(:,j)', r_T, num_iters, lambda);
    %    dkp(:, j) = W(:, end);
    %end
    
    %dkpuv = flipud(dkp);
    
    % new keypoints = old kpts + distance
    %S.t1.P = S.t0.P+dkpuv';
    
    % discard values with NaN return, if not already discarded
    %NaNidx = find(isnan(dkpuv(1,:)));
    %keep(NaNidx)=0;
    
    [keypoints,keep] = pointTracker(image);
    S.t1.P = flipud(keypoints);
    release(pointTracker);
    disp(['Points that were tracked: ',num2str(length(keep(keep>0)))])
    S.t1.P = double(round(S.t1.P(find(keep>0),:)));
    
    % delete keypoints and landmarkes that are discarded
    S.t0.P = S.t0.P(find(keep>0),:);
    S.t0.X = S.t0.X(find(keep>0),:);

    % break here if less than 15 keypoints are tracked
    if length(keep(keep>0))<15
        disp('Less than 15 keypoints tracked, execution stopped')
        break
    end
    
    S = estimaterelativePose(S, K);
    
    % attempt to recover scale factor
    %recover scaling factor
    %diff_old = diff(S{2,2});
    %diff_new = diff(S{1,2});
    %distances_old = sqrt(sum(diff_old.^2,2));
    %distances_new = sqrt(sum(diff_new.^2,2));
    %median_scale = median(distances_old./distances_new)
    %S{1,6}(1:3,4) =  S{1,6}(1:3,4)*median_scale;
    %S{1,6}
    %Position(:,i) = S{2,6}(1:3,4)+S{1,6}(1:3,4);
    
    disp('Current pose:')
    disp(S.t1.Pose)
    
    % debug
    debug = true;
    if (debug==true)
        debugplot(S, prev_image,image)
    end
    
    % set previous image to current image, needed for next iteration
    prev_image = image;
    
end
