% Parking Parameters

%choose if new landmarks shall be detected (otherwise only running with
%init landmarks)
global detectNewLnd;
detectNewLnd = true;
%rescale for cont operation
global cont_rescale
cont_rescale = 1;
% Harris Kernel Size
global HrKernel
HrKernel = 9;
% HArris Rescale
global HrScale
HrScale = 1;
% Harris Minimum Quality
global HrQuality
HrQuality = 0.000001;
% Minimum angle for new landmarks
global MinAngle
MinAngle = 1.5/180 * pi;
% Maxima suppression (discard if image points are too close)
global Suppression
Suppression = 2;
% Featue Matching Threshold for triangulation of new landmarks
global MatchThresholdCont
MatchThresholdCont = 100;
%Feature Matching Threshold for Initializaation
global MatchThresholdInit
MatchThresholdInit = 15;
%MAximum permitted reprojection Error (in Pixels??)
global MaxReprojError
MaxReprojError = 2;
%Minimum Points needed to continue execution, break otherwise
global MinPoints
MinPoints = 15;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Parameters for KLT
% patch radius
global r_T
r_T = 15;
% max number of iterations
global num_iters
num_iters = 50;
% max bidirectional error
global lambda
lambda = 5;
global numPyramids
numPyramids = 3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%RANSAC Parameters
global NumTrials
NumTrials = 100000;
global DistanceThreshold
DistanceThreshold = 0.001;
global InlierPercentage
InlierPercentage = 80;
global Confidence
Confidence = 99.99;