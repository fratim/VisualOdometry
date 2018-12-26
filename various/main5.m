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

% establish S data struct (cell), explained later (4.1)
% 4.1 Markovian idea: cell of information (denoted by S) is passed and updated continously,
% computation of S(t+1) solely depends on S(t)
% Cell struct S should contain: Keypoints (P), associated landmarks (X),
% set of candidate keypoints (C), candidate first observation (F),
% candidate first camera pose (T)
% 
% For the beginning, the VO pipeline can work with only the keypoints given by the initialization,
% afterwards the method of triangulating new landmarks can be mplemnted
% (this requires C, F and T)
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

% rescale factor for SIFT image processing
rescale = 1;
S = establishKptCorrespondencesHARRIS(S, img0, img1, rescale);

% 3.3 Relative pose estimation and triangulation of landmarks, use RANSAC 

S = estimaterelativePose(S, K);
%S{1,6}(1:3,4)=S{1,6}(1:3,4)*ground_truth(bootstrap_frames(2)+1,1)

% rescale groundtruth
    
% executed in triangulate relative pose
%S = triangulateLandmarkslinear(S, K); 
    
  % Debug plot results
debug = true;
if (debug)

    %plot only specific points
    pplot = 1:length(S{2,1}(:,1));
    %pplot = [1,2,3,4];

    subplot(2,2,1)
    imshow(img0)
    hold on
    scatter(S{2,1}(pplot,2),S{2,1}(pplot,1),'red')
    title('img0 matched features')

    subplot(2,2,2)
    imshow(img1)
    hold on
    scatter(S{1,1}(pplot,2),S{1,1}(pplot,1),'blue')
    title('img1 matched features')

    subplot(2,2,3)
    scatter3(S{1,2}(pplot,1),S{1,2}(pplot,2),S{1,2}(pplot,3),'blue')
    title('world points')

    % attempt to plot camera positions, not really nice yet
    hold on
    cam1 = plotCamera('Location',S{1,6}(1:3,4),'Orientation',S{1,6}(1:3,1:3),'Size',2);
    hold on
    cam2 = plotCamera('Location',S{2,6}(1:3,4),'Orientation',S{2,6}(1:3,1:3),'Size',2);

end

disp('Position estimate after initialization:')
disp(S{1,6})

% 3.4 Initialize VO pipeline with inlier keypoints and associated landmarks
    % This function must return P_0 (Keypoints) and X_0 (associated landmarks)
    
    % possible simply cut of the second row of the S cell
    % S = S(1,:);

%% Continous VO Pipeline (must be called in continous operation bodey)




%% Continuous operation
close all
range = (bootstrap_frames(2)+1):last_frame;
%only take every 5th frame as a keyframe
range = range(1):2:range(end);
Position = zeros(3,length(range));

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
    
    if (i == range(1))
        %load last picture used for bootstrap
        prev_image = im2uint8(rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))])));
    end
    
    % new timestep, theefore update S
    S(2,:) = S(1,:);
    S(1,1:6) = {P_t1,X_t1,C_t1,F_t1,T_t1, Pose_t1};
    
    % flip to meet x y convention
    keypoints = flipud(S{2,1}');
    r_T = 15;
    num_iters = 50;
    dkp = zeros(2,size(S{2,1},1));
    keep = zeros(1,size(S{2,1},1));
    lambda = 2;
    
    for j = 1:size(S{2,1},1)
        [W, keep(j)] = trackKLTRobustly(prev_image, image, keypoints(:,j)', r_T, num_iters, lambda);
        dkp(:, j) = W(:, end);
    end
    
    dkpuv = flipud(dkp);
    % new keypoints = old kpts + distance
    S{1,1} = S{2,1}+dkpuv';
    
    % discard values with NaN return, if no already discarded
    NaNidx = find(isnan(dkpuv(1,:)));
    keep(NaNidx)=0;
    disp(['Points that were tracked: ',num2str(length(keep(keep>0)))])
    S{1,1} = double(round(S{1,1}(find(keep>0),:)));
    
    % delete keypoints and landmarked that are discarded
    S{2,1} = S{2,1}(find(keep>0),:);
    S{2,2} = S{2,2}(find(keep>0),:);
    %landmarks stay the same
    %S{1,2} = S{2,2};
    
    % debug show points that were tracked
    clf
    imshow(image);
    hold on;
    plotMatches(1:size(S{1,1}, 1), S{1,1}', S{2,1}');
    hold off;

    % break here if less than 10 keypoints are tracked
    if length(keep(keep>0))<15
        disp('Less than 15 keypoints tracked, execution stopped')
        break
    end
    
    S = estimaterelativePose(S, K);
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
    disp(S{1,6})
    prev_image = image;
    
    % debug
    debug = true;
    if (debug)

        %plot only specific points
        pplot = 1:length(S{2,1}(:,1));
        %pplot = [1,5,25];

        subplot(2,2,1)
        imshow(img0)
        hold on
        scatter(S{2,1}(pplot,2),S{2,1}(pplot,1),'red')
        title('img0 matched features')

        subplot(2,2,2)
        imshow(img1)
        hold on
        scatter(S{1,1}(pplot,2),S{1,1}(pplot,1),'blue')
        title('img1 matched features')

        subplot(2,2,3)
        scatter3(S{1,2}(pplot,1),S{1,2}(pplot,2),S{1,2}(pplot,3),'blue')
        title('img1 matched features')

        % attempt to plot camera positions, not really nice yet
        hold on
        cam1 = plotCamera('Location',S{1,6}(1:3,4),'Orientation',S{1,6}(1:3,1:3),'Size',30);
        hold on
        cam2 = plotCamera('Location',S{2,6}(1:3,4),'Orientation',S{2,6}(1:3,1:3),'Size',30);

    end
    
    %imshow(image)
end
