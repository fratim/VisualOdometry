% Parking Parameters

% Harris Kernel Size
global HrKernel
HrKernel = 9;
% HArris Rescale
global HrScale
HrScale = 0.7;
% Harris Minimum Quality
global HrQuality
HrQuality = 0.00001;
% Minimum angle for new landmarks
global MinAngle
MinAngle = 1.5/180 * pi;
% Maxima suppression (discard if image points are too close)
global Suppression
Suppression = 2;
% Featue Matching Threshold for triangulation of new landmarks
global MatchThresholdCont
MatchThresholdCont = 15;
%Feature MAtching Threshold for Initializaation
global MatchThresholdInit
MatchThresholdInit = 15;
%MAximum permitted reprojection Error (in Pixels??)
global MaxReprojError
MaxReprojError = 2;
%Minimum Points needed to continue execution, break otherwise
global MinPoints
MinPoints = 15;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters for KLT
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