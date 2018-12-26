function S = triangulateLandmarkslinear(S, K)

    pt1 = [S{2,1}'; ones(1,length(S{2,1}))];
    pt2 = [S{1,1}'; ones(1,length(S{1,1}))];
    
    Mt1 = K*S{2,6};
    Mt2 = K*S{1,6};
    
    P = linearTriangulation(pt1,pt2,Mt1,Mt2);
    S{1,2} = P(1:3,:)';
    

end