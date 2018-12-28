function X = triangulateNewLandmarklinear(p1,p2,P1,P2,K)

    pt1 = [p1'; 1];
    pt2 = [p2'; 1];
    
    Mt1 = K*P1;
    Mt2 = K*P2;
    
    P = linearTriangulation(pt1,pt2,Mt1,Mt2);
    X = P(1:3,:)';    

end