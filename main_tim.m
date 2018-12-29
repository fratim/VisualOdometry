clear all
close all
clc
addpath(genpath('.'));
%% Setup
ds = 2; % 0: KITTI, 1: Malaga, 2: parking
debug = true;
kitti_path = 'kitti';
malaga_path = 'malaga-urban-dataset-extract-07';
parking_path = 'parking';

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
    bootstrap_frames = [0,4];
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    bootstrap_frames = [0,4];
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

%Bootstrapping 
S = bootstrap(img0,img1,K);

% debug mode: get rescale factor from groundtruth
% fit estimated pose to groundtruth, to recover scale factor
% if (debug == true)
%    S.t1.Pose(1:3,4) = S.t1.Pose(1:3,4)*ground_truth(bootstrap_frames(2)+4,1);
% end

% Debug plot results
debug = true;
if (debug == true)
    debugplot(S, img0, img1)   
    disp('Position estimate after initialization:')
    disp(S.t1.Pose)
end



%% Continuous operation

close all
range = (bootstrap_frames(2)+1):last_frame;
% possible only take every xth frame (right now every second)
%range = range(1):2:range(end);
%start = range(1)
%range = [start, start + 1, start + 2, start + 4, start + 5, start + 7, start + 8, start + 9]
%create matlab klt point tracker with parameters below
r_T = 20;
bs = 2*r_T+1;
num_iters = 50;
lambda = 1;

pointTracker = vision.PointTracker('NumPyramidLevels',1, ...
        'MaxBidirectionalError',lambda,'BlockSize',[bs,bs],'MaxIterations',num_iters);

%take second bootstrap image for initialization
prev_image = img1;
plot_count = 5;

%iterate through all frames from video
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
    
    % Do tracking from last to new frame
    [S,running] = continous_tracking(pointTracker,S,prev_image,image,K);

    % Check if enough features are available
    if(~running)
        break
    end
   
    % debug
    if (debug==true && plot)
        debugplot(S, prev_image,image)
        disp('Current pose:')
        disp(S.t1.Pose)
    end
    pause(0.1);
    
    % set previous image to current image, needed for next iteration
    prev_image = image;
    
end
