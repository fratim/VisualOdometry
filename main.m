clear all
close all
clc
addpath(genpath('.'));

%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
debug = true;
kitti_path = 'kitti';
malaga_path = 'malaga-urban-dataset-extract-07';
parking_path = 'parking';

if ds == 0
    run KittiParameters
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    run MalagaParameters
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
    run ParkingParameters
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
    bootstrap_frames = [0,2];
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    bootstrap_frames = [1,3];
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

global global_rescale

img0 = imresize(img0, global_rescale);
img1 = imresize(img1, global_rescale);

%Bootstrapping 
[S, success] = bootstrap(img0,img1,K);

if(~success)
    disp('Initialization not successfull!')
    return
end

%debug position output
debug = true;
if (debug == true)
    showMatchedFeatures(img0,img1,fliplr(S.t0.P),fliplr(S.t1.P))  
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

global r_T
global num_iters
global lambda
global numPyramids

bs = 2*r_T+1;

pointTracker = vision.PointTracker('NumPyramidLevels',numPyramids, ...
        'MaxBidirectionalError',lambda,'BlockSize',[bs,bs],'MaxIterations',num_iters);

%take second bootstrap image for initialization
prev_image = img1;
%plot eyery x images
plot_freq = 1;
plot_index = 6;

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
    
    %global rescale
    image = imresize(image, global_rescale);
    
    % debug
    if (debug==true && plot_index > (plot_freq+1))
        %debugplot(S, prev_image,image)
        showMatchedFeatures(prev_image,image,fliplr(S.t0.P),fliplr(S.t1.P))
        pause(0.01);
        plot_index = 0;
    end
    
    % Do tracking from last to new frame
    [S,running] = contTra(pointTracker,S,prev_image,image,K);

    % Check if enough features are available
    if(~running)
        break
    end
    
    % set previous image to current image, needed for next iteration
    prev_image = image;
    plot_index = plot_index + 1;
    
end
