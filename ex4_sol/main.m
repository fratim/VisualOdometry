clear all
close all
clc

num_scales = 3; % Scales per octave.
num_octaves = 5; % Number of octaves.
sigma = 1.0;
contrast_threshold = 0.04;
image_file_1 = 'images/img_1.jpg';
image_file_2 = 'images/img_2.jpg';
rescale_factor = 0.3; % Rescaling of the original image for speed.

images = {getImage(image_file_1, rescale_factor),...
    getImage(image_file_2, rescale_factor)};

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

figure; ax = axes;
showMatchedFeatures(images{1}, images{2}, kpt_matched_1, kpt_matched_2, ...
    'montage','Parent',ax);
title(ax, 'Candidate point matches');
legend(ax, 'Matched points 1','Matched points 2');
