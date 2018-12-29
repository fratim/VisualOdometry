% Parking Parameters

% Harris Kernel Size
HrKernel = 3;
% HArris Rescale
HrScale = 0.3;
% Harris Minimum Quality
HrQuality = 0.00001;
% Minimum angle for new landmarks
MinAngle = 1.5/180 * pi;
% Maxima suppression (discard if image points are too close)
suppression = 2;
% Featue Matching Threshold for triangulation of new landmarks
MatchThresholdCont = 99.9999;
%Feature MAtching Threshold for Initializaation
MatchThresholdInit = 15;
%MAximum permitted reprojection Error (in Pixels??)
MaxReprojError = 2;