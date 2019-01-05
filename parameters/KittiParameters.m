% Parking Parameters

% MAx reprojection error used for camera pose estimateion function
global MaxReprojErrorCameraPose
MaxReprojErrorCameraPose = 10;
global key_freq
key_freq = 3;
%choose if new landmarks shall be detected (otherwise only running with
%init landmarks)
global detectNewLnd;
detectNewLnd = true;
%global rescale, before any operation
global cont_rescale
cont_rescale = 1 ;%10.75;
% Harris Kernel Size
global HrKernel
HrKernel = 9;
% HArris Rescale
global HrScale
HrScale = 0.75;
% Harris Minimum Quality
global HrQuality
HrQuality = 0.001;
% Minimum angle for new landmarks
global MinAngle
MinAngle = 0.1/180 * pi;
% Feature Block Patch Size
global BlockSize
BlockSize = 17;
% Maxima suppression (discard if image points are too close)
global Suppression
Suppression = 11;
% Featue Matching Threshold for triangulation of new landmarks
global MatchThresholdCont
MatchThresholdCont = 90;
%Feature MAtching Threshold for Initializaation
global MatchThresholdInit
MatchThresholdInit = 90;
%MAximum permitted reprojection Error (in Pixels??)
global MaxReprojError
MaxReprojError = 2;
%Minimum Points needed to continue execution, break otherwise
global MinPoints
MinPoints = 15;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Parameters for KLT
%patch radius
global r_T
r_T = 15;
% max number of iterations
global num_iters
num_iters = 1000;
%max bidirectional error
global lambda
lambda = 1;
global numPyramids
numPyramids = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%RANSAC Parameters
global NumTrials
NumTrials = 10000;
global DistanceThreshold
DistanceThreshold = 1;
global InlierPercentage
InlierPercentage = 80;
global Confidence
Confidence = 99;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global kptadd
kptadd = 300;