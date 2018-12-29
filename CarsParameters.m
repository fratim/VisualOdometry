% Parking Parameters

% Harris Kernel Size
HrKernel = 9;
% HArris Rescale
HrScale = 0.7;
% Harris Minimum Quality
HrQuality = 0.00001;
% Minimum angle for new landmarks
MinAngle = 1.5/180 * pi;
% Maxima suppression (discard if image points are too close)
suppression = 2;
% Featue Matching Threshold for triangulation of new landmarks
MatchThresholdCont = 15;
%Feature MAtching Threshold for Initializaation
MatchThresholdInit = 15;
%MAximum permitted reprojection Error (in Pixels??)
MaxReprojError = 2;