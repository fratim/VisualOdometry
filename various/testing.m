K = rand(1)*eye(3);


inv(K)'*[0 0 0; 0 0 -1; 0 1 0]*eye(3)*inv(K)