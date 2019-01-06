% Parking Parameters

% MAx reprojection error used for camera pose estimateion function
global MaxReprojErrorCameraPose
MaxReprojErrorCameraPose = 1;
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
HrKernel = 5;
% HArris Rescale
global HrScale
HrScale = 0.75;
% Harris Minimum Quality
global HrQuality
HrQuality = 0.0001;
% Minimum angle for new landmarks
global MinAngle
MinAngle = 0.2/180 * pi;
% Feature Block Patch Size
global BlockSize
BlockSize = 17;
% Maxima suppression (discard if image points are too close)
global Suppression
Suppression = 15;
% Featue Matching Threshold for triangulation of new landmarks
global MatchThresholdCont
MatchThresholdCont = 70;
%Feature MAtching Threshold for Initializaation
global MatchThresholdInit
MatchThresholdInit = 70;
%MAximum permitted reprojection Error (in Pixels??)
global MaxReprojError
MaxReprojError = 5;
%Minimum Points needed to continue execution, break otherwise
global MinPoints
MinPoints = 10;
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
numPyramids = 10;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%RANSAC Parameters
global NumTrials
NumTrials = 10000;
global DistanceThreshold
DistanceThreshold = 3;
global InlierPercentage
InlierPercentage = 60;
global Confidence
Confidence = 99;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global kptadd
kptadd = 200;
global PGoal
PGoal = 100;
global n_crit
n_crit = 20;
global kptmax
kptmax = 150;
global kptaddalways
kptaddalways = 10 ;
global min_boot
min_boot = 16;