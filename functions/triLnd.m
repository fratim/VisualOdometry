function [p0,p1,X] = triLnd(S,R,T,p0,p1,firstPose)

    global MaxReprojError
    global NumTrials
    global DistanceThreshold
    global InlierPercentage
    global Confidence
    
    %Strange Camera matrix shit for matlab (see documentation of
    %cameraMatrix)
    R_n = S.t1.Pose(1:3,1:3);
    T_n = S.t1.Pose(1:3,4)';
    
    R_o = firstPose(1:3,1:3);
    T_o = firstPose(1:3,4)';
    
    R_0= R_o';
    T_0= -T_o*R_0;
   
    R_1= R_n';
    T_1= -T_n*R_1;
    
    R_c = R';
    T_c = -T*R_c;
    
    camM1 = cameraMatrix(S.K,R_0,T_0);
    camM2 = cameraMatrix(S.K,R_1,T_1);
    
    %correct pose
    %Triangulate points
    [worldP,reprojectionErrors] = triangulate(p0,p1,camM1,camM2);
    keep = reprojectionErrors < MaxReprojError; 
    
    %delete points behind camera or that are too far away
    %find transform to current camera pose coordinate system
    T = [S.t1.Pose;zeros(1,3),1];
    worldP_cameraframe = inv(T)*[worldP';ones(1,size(worldP,1))];
    worldP_cameraframe = worldP_cameraframe(1:3,:)';
    
    %z bigger than 0
    keep(find(worldP_cameraframe(:,3)<0))=0;
    
    % distance in z smaller than 40
    keep(find(worldP_cameraframe(:,3)>40))=0;
    
    %only keep points that meet criteria
    worldP = worldP(keep,:);
    p0=p0(keep,:);
    p1=p1(keep,:);
    
    X=worldP;
   

    
end