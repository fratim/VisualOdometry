function S = estimaterelativePose(S, K)
    
    % inliersIndex and status can be used for debugging, no use in normal
    % mode
    [F,inliersIndex,status] = estimateFundamentalMatrix(fliplr(S.t0.P(:,:)),...
    fliplr(S.t1.P(:,:)),'Method','RANSAC',...
    'NumTrials',100000,'DistanceThreshold',0.001,'InlierPercentage',90,'Confidence',99.99);

    %calculate essential Matrix E
    E = K'*F*K;
    [Rots,u3] = decomposeEssentialMatrix(E);

    kpt_matched_t1 = [transpose(S.t0.P(:,:));ones(1,length(S.t0.P(:,:)))];
    kpt_matched_t2 = [transpose(S.t1.P(:,:));ones(1,length(S.t1.P(:,:)))];
    
    M0=K*S.t0.Pose;
    [R,T] = disambiguateRelativePose(Rots,u3,kpt_matched_t1,kpt_matched_t2,K,K,M0);
    
    disp('R and T estimated over last iteration: ')
    disp([R,T])
    
    T_new = [S.t0.Pose;zeros(1,3),1]*[R,T;zeros(1,3),1];
    S.t1.Pose = T_new(1:3,1:4);
    
    S = triangulateLandmarkslinear(S, K);
    
end