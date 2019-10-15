function poses = odometryFromEdges(edges,nrNodes,verbosity,initialGuess)
% Builds the odometric path from the delta poses encoded in "edges"
% The function assumes that the first numNodes-1 edges are the *ordered*
% spanning path (i.e., edges are 1->2, 2->3, ..., numNodes-1->numNodes)

if (nargin < 3) verbosity = 0; end

poses = zeros(nrNodes,3);
for k = 2:nrNodes
  if (edges(k-1,2) ~= edges(k-1,1)+1) || (edges(k-1,1) ~= k-1)  % odometric edges first
    edges(k-1,:),k-1, error('wrong odometric edges')
  end
  delta_pose = edges(k-1,3:end);
  poses(k,:) = poseAdd(poses(k-1,:) , delta_pose);
end
if verbosity>0
  figure(200); 
  plot(poses(:,1),poses(:,2), '-m'); hold on
  title('Initial guess from odometry')
end

if nargin > 3 % also provided another initial guess
  plot(initialGuess(:,1),initialGuess(:,2), '--b');
  if norm(initialGuess(1,:))>0
    disp('provided initial guess does not have the first node at the origin')
  end
  title('Initial guess from odometry (magenta) VS declared initial guess (blue)');
end

m = size(edges,1);
if verbosity>1
  for k = nrNodes:m % loop closures
    id1 = edges(k,1);
    id2 = edges(k,2);
    plot(poses([id1 id2],1),poses([id1 id2],2), '-k'); hold on
  end
end
