function [newP, newX] = enforceBlocksKpt(newP, newX, newQuality,kpt_needed, xmax, ymax)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

nBlocksX = 15; %blocksize in Px
nBlocksY = 5;

blockSizeX = floor(xmax/nBlocksX);
blockSizeY = floor(ymax/nBlocksY);

%perfect amount of points per block (equally distributed)
pointsperblockopt = round(kpt_needed/(nBlocksX*nBlocksY));

%(maximum of pooints added (maybe twice the optimum?)
pointsBlockMax = round(pointsperblockopt*1.5);

% sort all points by quality
[~,Idx_sorted] = sort(newQuality,'descend');
newP = newP(Idx_sorted,:);
newX = newX(Idx_sorted,:);


for i = 1:nBlocksX
    pxmin = i*blockSizeX-blockSizeX+1;
    pxmax = i*blockSizeX;
    for j = 1:nBlocksY
        pymin = j*blockSizeY-blockSizeY+1;
        pymax = j*blockSizeY;
        
        idx_inblock = find( newP(:,1)>pxmin & newP(:,1)<pxmax &...
                            newP(:,2)>pymin & newP(:,2)<pymax);
                        
        if length(idx_inblock)>pointsBlockMax
           
           newP(idx_inblock(pointsBlockMax+1:end),:)=[];
           newX(idx_inblock(pointsBlockMax+1:end),:)=[];
        end

    end
end

end

