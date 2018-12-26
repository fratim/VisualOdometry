function S_data = establishKptCorrespondencesSIFT(S_data, img0, img1, rescale)

    num_scales = 3; % Scales per octave.
    num_octaves = 5; % Number of octaves.
    sigma = 1.0;
    contrast_threshold = 0.04;
    rescale_factor = rescale; % Rescaling of the original image for speed.
        
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
    
    S_data{1,1}=kpt_matched_2/rescale;
    S_data{2,1}=kpt_matched_1/rescale;
end