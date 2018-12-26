function S = estimaterelativePose(S, K)
    
    % inliersIndex and status can be used for debugging, no use in normal
    % mode
    [F,inliersIndex,status] = estimateFundamentalMatrix(fliplr(S{2,1}(:,:)),...
    fliplr(S{1,1}(:,:)),'Method','RANSAC',...
    'NumTrials',100000,'DistanceThreshold',0.001,'InlierPercentage',90,'Confidence',99.99);

    %calculate essential Matrix E
    E = K'*F*K;
    [Rots,u3] = decomposeEssentialMatrix(E);

    kpt_matched_t1 = [transpose(S{2,1}(:,:));ones(1,length(S{2,1}(:,:)))];
    kpt_matched_t2 = [transpose(S{1,1}(:,:));ones(1,length(S{1,1}(:,:)))];
    
    M0=K*S{2,6};
    [R,T] = disambiguateRelativePose(Rots,u3,kpt_matched_t1,kpt_matched_t2,K,K,M0);
    
    disp('R and T estimated over last iteration: ')
    disp([R,T])
    
    T_new = [S{2,6};zeros(1,3),1]*[R,T;zeros(1,3),1];
    S{1,6} = T_new(1:3,1:4);
    
    S = triangulateLandmarkslinear(S, K);
    
end