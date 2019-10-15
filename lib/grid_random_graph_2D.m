function graph = grid_random_graph_2D(nNodes, varargin)
% Creates a grid random graph, with a pose associated to nodes and relative pose
% measurements associated to edges
% 
% INPUTS:
% - nNodes: number of nodes in the graph
%
% OPTIONS:
% - 'Noise': 'gaussian' or 'uniform'
% - 'RotationStd': std of rotation noise
% - 'Scale': multiplies the translation part by a scale factor
% - 'TranslationStd': std of translation noise
% - 'LoopClosureProbability': probability of creating a loop closure between 2 nodes
%
% OUTPUTS:
% - Graph structure containing
%   - model: 'grid'
%   - scale: input scale factor
%   - edges: m x 11 matrix describing edges (in g2o format): each row, describe edge (i,j) 
%            and is in the form [i j pij Ixx Ixy Ixth Iyy Ixth Ithth] where i
%            and j are the node id, pij = [x y th] is the 2D measured relative pose and
%            I describe the entries of the information matrix describing
%            measurement noise
%   - thetha_gt: ground truth orientation of the nodes generating the measurements
%   - poses_gt: ground truth poses of the nodes generating the measurements
%   - pose_estimate: pose estimate (odometry) obtained by concatenating measurements
%   - sigma_R: std of rotation noise
%   - sigma_t: std of tranlation noise
%

% Author: Luca Carlone
% Date: 2015-1-1
% Institution: Massachusetts Institute of Technology
%
% Edited: Pasquale Antonante
% Date: 2018-10-18
% Institution: Massachusetts Institute of Technology

params = inputParser;
params.CaseSensitive = false;
% Noise: guassian or uniform?
params.addParameter('Noise', 'gaussian', ...
    @(x) ischar(x) && ...
    any(strcmpi({'gaussian', 'uniform'}, x)));
% Rotation Parameters
params.addParameter('RotationStd', 0.01, ...
    @(x) isnumeric(x));
% Position (x-y) parameters
params.addParameter('Scale', 1, ...
    @(x) isnumeric(x));
params.addParameter('TranslationStd', 0.1, ...
    @(x) isnumeric(x));
params.addParameter('LoopClosureProbability', 0.3, ...
    @(x) isnumeric(x) && x>= 0 && x<=1);
% Parse 'em all!
params.parse(varargin{:});

factors = factor(nNodes);
nrRows = prod(factors(1:end-1));
nrCols = factors(end); 
assert(nrCols*nrRows == nNodes, 'Wrong factorization')
assert(nrRows ~= 1 && nrCols~= 1, 'The number of nodes cannot be a prime number')

% Utility vars
sigmaT = params.Results.TranslationStd;
sigmaR = params.Results.RotationStd;
graph_scale = params.Results.Scale;
probLC = params.Results.LoopClosureProbability;
epsilon = 0.01;

% create GT poses
posesGT = zeros(nrRows*nrCols,3);
corners = [];
nNodes = 0;
for i=1:nrRows
  for j=1:nrCols
    nNodes = nNodes+1; % we add a new node
    if i==1 && j ==1 
      th = 0;
    else
      th = 2*pi*rand - pi; % random orientation
    end
    if rem(i,2)==1 % odd rows
      posesGT(nNodes,:) = [j, i, th];
    else
      posesGT(nNodes,:) = [nrCols-j+1, i, th];
    end
    if i==nrRows || j==1 || j==nrCols
      corners = [corners nNodes];
    end
  end
end

posesGT(:,1:2) = graph_scale * posesGT(:,1:2); % scale only the cartesian part

% create odometric edges
m = nNodes-1;
for i=1:m
  id1 = i;
  id2 = i+1;
  edges(i,1:2) = [id1 id2];
  p1 = posesGT(id1, 1:3); % gt positions
  p2 = posesGT(id2, 1:3);
  edges(i, 3:5) = poseSubNoisy(p2 , p1, sigmaT, sigmaR);
  edges(i, 6:11) = [1/sigmaT^2 0 0 1/sigmaT^2 0 1/sigmaR^2];
end

% create loop closures
for id1=[2:nNodes]%[1:nNodes]
  p1 = posesGT(id1, 1:3); % gt positions
  for id2=[id1+1:nNodes] %% we look for all possible neighbors (unit distance)
    p2 = posesGT(id2, 1:3);
    if ( norm(p1(1:2) - p2(1:2)) < params.Results.Scale + epsilon && norm(p1(1:2) - p2(1:2)) > epsilon ) % unit distance but not the same node
      edge = [id1 id2];
      assert(id1 ~=id2, 'Coincident nodes?')
 
      if rand<probLC && findUndirectedEdge(edge, edges) == 0 % add edge with some probability if it is not there yet
        m = m+1;
        edges(m,1:2) = [id1 id2];
        edges(m, 3:5) = poseSubNoisy(p2 , p1, sigmaT, sigmaR);
        edges(m, 6:11) = [1/sigmaT^2 0 0 1/sigmaT^2 0 1/sigmaR^2];
      end
    end
  end
end

% Anchoring the first node to the identity
posesGT = anchorFirstNode(posesGT);
pose_estimate = odometryFromEdges(edges,nNodes);

graph = struct(...
  'model', 'grid', ...
  'format', '2d', ...
  'scale', params.Results.Scale, ...
  'edges', edges, ...
  'poses_gt', posesGT, ...
  'pose_estimate', pose_estimate, ...
  'sigma_R', sigmaR, ...
  'sigma_t', sigmaT ...
);
graph.measurements = posegraphMeasurementMatrix(graph);

end