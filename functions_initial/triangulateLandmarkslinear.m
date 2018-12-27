function S = triangulateLandmarkslinear(S, K)

    pt1 = [S.t0.P'; ones(1,length(S.t0.P))];
    pt2 = [S.t1.P'; ones(1,length(S.t1.P))];
    
    Mt1 = K*S.t0.Pose;
    Mt2 = K*S.t1.Pose;
    
    P = linearTriangulation(pt1,pt2,Mt1,Mt2);
    S.t1.X = P(1:3,:)';    

end