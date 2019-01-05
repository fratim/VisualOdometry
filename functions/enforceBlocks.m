function [S] = enforceBlocks(S,xmax,ymax)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

nBlocksX = 10; %blocksize in Px
nBlocksY = 3;

blockSizeX = floor(xmax/nBlocksX);
blockSizeY = floor(ymax/nBlocksY);

maxEachBlock = 30;

for i = 1:nBlocksX
    pxmin = i*blockSizeX-blockSizeX+1;
    pxmax = i*blockSizeX;
    for j = 1:nBlocksY
        pymin = j*blockSizeY-blockSizeY+1;
        pymax = j*blockSizeY;
        
        idx_inblock = find( S.t1.P(:,1)>pxmin & S.t1.P(:,1)<pxmax &...
                            S.t1.P(:,2)>pymin & S.t1.P(:,2)<pymax);
                        
        if length(idx_inblock)>maxEachBlock
           S.t1.P(idx_inblock(maxEachBlock+1:end),:)=[];
           S.t1.X(idx_inblock(maxEachBlock+1:end),:)=[];
        end
    end
end

end

