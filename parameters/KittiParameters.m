% Parking Parameters
%global rrscale, before any operation
global global_rescale
global_rescale = 1;
% Harris Kernel Size
global HrKernel
HrKernel = 17;
% HArris Rescale
global HrScale
HrScale = 1;
% Harris Minimum Quality
global HrQuality
HrQuality = 0.001;
% Minimum angle for new landmarks
global MinAngle
MinAngle = 1/180 * pi;
% Maxima suppression (discard if image points are too close)
global Suppression
Suppression = 10;
% Featue Matching Threshold for triangulation of new landmarks
global MatchThresholdCont
MatchThresholdCont = 15;
%Feature MAtching Threshold for Initializaation
global MatchThresholdInit
MatchThresholdInit = 15;
%MAximum permitted reprojection Error (in Pixels??)
global MaxReprojError
MaxReprojError = 10;
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
lambda = 2;
global numPyramids
numPyramids = 3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%RANSAC Parameters
global NumTrials
NumTrials = 2000;
global DistanceThreshold
DistanceThreshold = 1;
global InlierPercentage
InlierPercentage = 70;
global Confidence
Confidence = 99.99;