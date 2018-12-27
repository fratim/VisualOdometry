function S = estimaterelativePose(S, K)
    
    % inliersIndex and status can be used for debugging, no use in normal
    % mode
    [F,inliersIndex,status] = estimateFundamentalMatrix(fliplr(S{2,1}(:,:)),...
    fliplr(S{1,1}(:,:)),'Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1e-4);

    [Rots,u3] = decomposeEssentialMatrix(F);

    kpt_matched_t1 = [transpose(S{2,1}(:,:));ones(1,length(S{2,1}(:,:)))];
    kpt_matched_t2 = [transpose(S{1,1}(:,:));ones(1,length(S{1,1}(:,:)))];
    
    [R,T] = disambiguateRelativePose(Rots,u3,kpt_matched_t1,kpt_matched_t2,K,K);
    S{1,6} = [R,T];
    
end