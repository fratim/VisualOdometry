function [kpt_new, quality_new] = enforceBlocksKptNew(kpt_new, quality_new,kpt_needed, xmax, ymax)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

nBlocksX = 1; %blocksize in Px
nBlocksY = 1;

blockSizeX = floor(xmax/nBlocksX);
blockSizeY = floor(ymax/nBlocksY);


%perfect amount of points per block (equally distributed)
pointsperblockopt = round(kpt_needed/(nBlocksX*nBlocksY));

%(maximum of pooints added (maybe twice the optimum?)
pointsBlockMax = round(pointsperblockopt*1.5);

% sort all points by quality
[quality_new,Idx_sorted] = sort(quality_new,'descend');
kpt_new = kpt_new(Idx_sorted,:);



for i = 1:nBlocksX
    pxmin = i*blockSizeX-blockSizeX+1;
    pxmax = i*blockSizeX;
    for j = 1:nBlocksY
        pymin = j*blockSizeY-blockSizeY+1;
        pymax = j*blockSizeY;
        
        idx_inblock = find( kpt_new(:,1)>pxmin & kpt_new(:,1)<pxmax &...
                            kpt_new(:,2)>pymin & kpt_new(:,2)<pymax);
                        
        if length(idx_inblock)>pointsBlockMax
           
           kpt_new(idx_inblock(pointsBlockMax+1:end),:)=[];
           quality_new(idx_inblock(pointsBlockMax+1:end),:)=[];
        end

    end
end

end

