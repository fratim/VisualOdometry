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
    imgs = [];
    bootstrap_frames = [0,3];
    for i=bootstrap_frames(1):bootstrap_frames(2)       
        img = imread([kitti_path '/00/image_0/' ...
            sprintf('%06d.png',i)]);
        imgs = [imgs;{img}];
    end
elseif ds == 1
    imgs = [];
    bootstrap_frames = [0,3];
    for i=bootstrap_frames(1):bootstrap_frames(2) 
        img = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i+1).name]));
        imgs = [imgs;{img}];
    end

elseif ds == 2
    imgs = [];
    bootstrap_frames = [0,3];
    for i=bootstrap_frames(1):bootstrap_frames(2) 
        img = rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)]));
        imgs = [imgs;{img}];
    end
else
    assert(false);
end

%Bootstrapping 
[S, success] = bootstrap(imgs,K);

if(~success)
    disp('Initialization not successfull!')
    return
end

%debug position output
debug = true;
if (debug == true)
    %debugplot(S, img0,img1)
    %showMatchedFeatures(cell2mat(imgs(bootstrap_frames(1)+1)),cell2mat(imgs(bootstrap_frames(2)+1)),fliplr(S.ti.P),fliplr(S.t1.P))  
    %disp('Position estimate after initialization:')
    disp(S.t1.Pose)
end

%% Continuous operation
img1 = cell2mat(imgs(bootstrap_frames(2)+1));
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
global cont_rescale
global key_freq

bs = 2*r_T+1;

pointTracker = vision.PointTracker('NumPyramidLevels',numPyramids, ...
        'MaxBidirectionalError',lambda,'BlockSize',[bs,bs],'MaxIterations',num_iters);

%take second bootstrap image for initialization
prev_image = img1;
%S.t0.P = S.t0.P*cont_rescale;
%S.t1.P = S.t1.P*cont_rescale;

%plot eyery x images
plot_freq = 1;
plot_index = 6;

keycounter = 0;

land_hist=zeros(20,1);
traj=[];
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
    %image = imresize(image, cont_rescale);
    
    disp(['new iteration, points tracked: ',num2str(length(S.t1.P))]);
    
    % new timestep, therefore update S
    S.t0 = S.t1;
    %S.ti.X(:,1:3)=S.ti.X(:,2:4);
    %S.ti.Y(:,1:3)=S.ti.Y(:,2:4);
    
    % Do tracking from last to new frame
    [S,running] = contTra(pointTracker,S,prev_image,image);

    % Check if enough features are available
    if(~running)
        break
    end
    
    traj = [traj; S.t1.Pose(1:3,4)'];
    land_hist(1:19)=land_hist(2:20);
    land_hist(20)= size(S.t1.X,1);
    % debug
    if (debug==true)% && plot_index > (plot_freq+1))
        %debugplot(S, prev_image,image)
        %p_o = [S.ti.X(:,3) S.ti.Y(:,3)];
        %p_n = [S.ti.X(:,4) S.ti.Y(:,4)];
        %showMatchedFeatures(prev_image,image,p_o,p_n)
        pause(0.01);      
        plot_frame(S,traj,land_hist,image);
        plot_index = 0;
    end
    
    % set previous image to current image, needed for next iteration
    prev_image = image;
    plot_index = plot_index + 1;
    
end
