function [posesGT] = anchorFirstNode(posesGT)
%ANCHORFIRSTNODE Anchor the first node to the identity
anchor = posesGT(1,:)';
for i=1:size(posesGT,1)
    posesGT(i,:) = poseSubNoisy(posesGT(i,:)' , anchor)';
end
end

