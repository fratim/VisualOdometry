% Parking Parameters


global key_freq
key_freq=3;
%choose if new landmarks shall be detected (otherwise only running with
%init landmarks)
global detectNewLnd;
detectNewLnd = true;
%global rrscale, before any operation
global cont_rescale
cont_rescale = 1 ;%10.75;
% Harris Kernel Size
global HrKernel
HrKernel = 5;
% HArris Rescale
global HrScale
HrScale = 0.5;
% Harris Minimum Quality
global HrQuality
HrQuality = 0.01;
% Minimum angle for new landmarks
global MinAngle
MinAngle = 1/180 * pi;
% Feature Block Patch Size
global BlockSize
BlockSize = 17;
% Maxima suppression (discard if image points are too close)
global Suppression
Suppression = 1;
% Featue Matching Threshold for triangulation of new landmarks
global MatchThresholdCont
MatchThresholdCont = 60;
%Feature MAtching Threshold for Initializaation
global MatchThresholdInit
MatchThresholdInit = 60;
%MAximum permitted reprojection Error (in Pixels??)
global MaxReprojError
MaxReprojError = 30;
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
num_iters = 200;
% max bidirectional error
global lambda
lambda = 3;
global numPyramids
numPyramids = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%RANSAC Parameters
global NumTrials
NumTrials = 5000;
global DistanceThreshold
DistanceThreshold = 5;
global InlierPercentage
InlierPercentage = 80;
global Confidence
Confidence = 99;