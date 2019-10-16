function posesMat3xN = pose2DvectToMat(poseVect3N)
% Transforms a vector describing a set of poses in the form
% poseVect3N = [x1 y1 x2 y2 ... xN yN theta1 theta2 ... thetaN]
% into a 3xN matrix with i-th row equal to [xi yi thetai]

nrNodes = length(poseVect3N)/3;
posesMat3xN = zeros(nrNodes,3); % preallocation
for i=1:nrNodes
  posesMat3xN(i,1:3) = [poseVect3N(2*i-1:2*i)'  poseVect3N(2*nrNodes+i)];
end