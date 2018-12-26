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

% 3.1 manually select two frames I_0 and I_1
    % this is done in the Bootstrap part, when introducing the 'bootstap_frames' vector
% 3.2 Establish keypoint correspondences (using KLT or patch matching)

    num_scales = 3; % Scales per octave.
    num_octaves = 5; % Number of octaves.
    sigma = 1.0;
    contrast_threshold = 0.04;
    image_file_1 = 'images/img_1.jpg';
    image_file_2 = 'images/img_2.jpg';
    rescale_factor = 0.3; % Rescaling of the original image for speed.
        
    img0 = im2double(imresize(img0,...
        rescale_factor));
    img1 = im2double(imresize(img1,...
        rescale_factor));
    
    images = {img0, img1};

    kpt_locations = cell(1, 2);
    descriptors = cell(1, 2);

    for img_idx = 1:2
        image_pyramid = computeImagePyramid(images{img_idx}, num_octaves);
        blurred_images = computeBlurredImages(image_pyramid, num_scales, sigma);
        DoGs = computeDoGs(blurred_images);
        tmp_kpt_locations = extractKeypoints(DoGs, contrast_threshold);
        [descriptors{img_idx}, kpt_locations{img_idx}] =...
            computeDescriptors(blurred_images, tmp_kpt_locations);
    end

    indexPairs = matchFeatures(descriptors{1}, descriptors{2},...
    'MatchThreshold', 100, 'MaxRatio', 0.7, 'Unique', true);

    kpt_matched_1 = fliplr(kpt_locations{1}(indexPairs(:,1), :));
    kpt_matched_2 = fliplr(kpt_locations{2}(indexPairs(:,2), :));

    %figure; ax = axes;
    %showMatchedFeatures(images{1}, images{2}, kpt_matched_1, kpt_matched_2, ...
    %'montage','Parent',ax);
    %title(ax, 'Candidate point matches');
    %legend(ax, 'Matched points 1','Matched points 2');

% 3.3 Relative pose estimation and triangulation of landmarks, use RANSAC 

    [F,inliersIndex,status] = estimateFundamentalMatrix(kpt_matched_1,...
    kpt_matched_2,'Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1e-4);

    [Rots,u3] = decomposeEssentialMatrix(F);
    
    kpt_matched_1 = [kpt_matched_1';ones(1,length(kpt_matched_1))];
    kpt_matched_2 = [kpt_matched_2';ones(1,length(kpt_matched_2))];
    
    K = rand(1)*[1 0 1; 0 1 0; 0 0 1]; % K does not matter to the result, as K1 = K2, but why exactley is that?
    [R,T] = disambiguateRelativePose(Rots,u3,kpt_matched_1,kpt_matched_2,K,K);
    
    position = T;

% 3.4 Initialize VO pipeline with inlier keypoints and associated landmarks
    % This function must return P_0 (Keypoints) and X_0 (associated landmarks)

%% Continous VO Pipeline (must be called in continous operation bodey)
% Markovian idea: cell of information (denoted by S) is passed and updated continously,
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
position = zeros(3,length(range));

figure; ax = axes;

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
        
        num_scales = 3; % Scales per octave.
        num_octaves = 5; % Number of octaves.
        sigma = 1.0;
        contrast_threshold = 0.04;
        rescale_factor = 0.3; % Rescaling of the original image for speed.

        if (bootstrap_frames(2)+1 == i)
            prev_img = image;
        end
        
        img0 = im2double(imresize(prev_img,...
            rescale_factor));
        img1 = im2double(imresize(image,...
            rescale_factor));
            
        
        images = {img0, img1};

        kpt_locations = cell(1, 2);
        descriptors = cell(1, 2);

        for img_idx = 1:2
            image_pyramid = computeImagePyramid(images{img_idx}, num_octaves);
            blurred_images = computeBlurredImages(image_pyramid, num_scales, sigma);
            DoGs = computeDoGs(blurred_images);
            tmp_kpt_locations = extractKeypoints(DoGs, contrast_threshold);
            [descriptors{img_idx}, kpt_locations{img_idx}] =...
                computeDescriptors(blurred_images, tmp_kpt_locations);
        end

        indexPairs = matchFeatures(descriptors{1}, descriptors{2},...
        'MatchThreshold', 100, 'MaxRatio', 0.7, 'Unique', true);

        kpt_matched_1 = fliplr(kpt_locations{1}(indexPairs(:,1), :));
        kpt_matched_2 = fliplr(kpt_locations{2}(indexPairs(:,2), :));
        
        length(kpt_matched_1)

        showMatchedFeatures(images{1}, images{2}, kpt_matched_1, kpt_matched_2, ...
        'montage','Parent',ax);
        %title(ax, 'Candidate point matches');
        %legend(ax, 'Matched points 1','Matched points 2');
        
        % 3.3 Relative pose estimation and triangulation of landmarks, use RANSAC 

        [F,inliersIndex,status] = estimateFundamentalMatrix(kpt_matched_1,...
        kpt_matched_2,'Method','RANSAC',...
        'NumTrials',2000,'DistanceThreshold',1e-3);

        [Rots,u3] = decomposeEssentialMatrix(F);

        kpt_matched_1 = [kpt_matched_1';ones(1,length(kpt_matched_1))];
        kpt_matched_2 = [kpt_matched_2';ones(1,length(kpt_matched_2))];

        K = rand(1)*[1 0 1; 0 1 0; 0 0 1]; % K does not matter to the result, as K1 = K2, but why exactley is that?
        [R,T] = disambiguateRelativePose(Rots,u3,kpt_matched_1,kpt_matched_2,K,K);

        position(:,i+1) = T+position(:,i);

    else
        assert(false);
    end
    % Makes sure that plots refresh.    
    pause(0.01);
    
    prev_img = image;
end
