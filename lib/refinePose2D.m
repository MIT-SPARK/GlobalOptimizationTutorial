function poseEst = refinePose2D(edges, thEst, posesInit)

isDebug = 0;

m = size(edges,1);
nrNodes = length(thEst); % thEst is the orientation estimate
n = nrNodes-1; % number of observable nodes

J = sparse(spalloc(4*m, 3*nrNodes, 6*4*m)); % f(x) in (4*m) x 3*nrNodes
% since each measurement includes a Cartesian measurement in R^2 and 
% a rotation measurement (converted to R^2 vector). The unknown x contains
% 3 variables per node (x,y,th). Each row of the Jacobian involves 2 nodes
% and can have at most 6 nonzero quantities
fhat = sparse(zeros(4*m,1));
Mij = [-1 0; 0 -1];
cost = 1e+20; % initial cost set to infinite
% Initial guess
if  nargin < 3
  poseEst = [zeros(2*nrNodes,1); thEst]; 
else
  poseEst = [reshape(posesInit(:,1:2)',2*nrNodes,1); posesInit(:,3)]; % written as a vector, with xy positions first, then theta
end

nonAnchorInds = [3:2*nrNodes,2*nrNodes+2:3*nrNodes];
JR = spalloc(4*m, nrNodes, 2*4*m); % 2 nonzero elements per row
JC = spalloc(4*m, 2*nrNodes, 4*4*m); % 4 nonzero elements per row

for iter=1:10 
  %% linerize
  costPrec = cost;
  for k=1:m  
    tic
    id1 = edges(k,1);
    id2 = edges(k,2);
    Ri = rot2D(poseEst(2*nrNodes+id1));
    Rj = rot2D(poseEst(2*nrNodes+id2));
    Rij = rot2D(edges(k,5));
    Deltaij = edges(k,3:4)';
    DeltaijMat = [Deltaij(1)  -Deltaij(2); Deltaij(2)  Deltaij(1)];
    
    id1c = blockToMatIndices(id1,2);
    p1 = poseEst(id1c);
    id2c = blockToMatIndices(id2,2);
    p2 = poseEst(id2c);
    
    fcart = p2 - p1 - DeltaijMat * Ri(:,1);
    frot  = Rij * Ri(:,1) + Mij * Rj(:,1);   
    
    %% here we do not need the 1/2, since is [cos sin] instead of rotation
    inf_th = sqrt ( edges(k,11) ); % inverse of std for rotation measurements 
    inf_cart = sqrt ( edges(k,6) ); % inverse of std for position measurements 
    
    % each measurement f in R^4 is f=[fcart frot]
    rowInd = blockToMatIndices(k,4);
    fhat(rowInd) = [inf_cart * fcart; inf_th * frot];
 
    JR(rowInd,id1) = [ - inf_cart * DeltaijMat * Ri(:,2); 
                         inf_th * Rij * Ri(:,2) ];
    JR(rowInd,id2) = [ zeros(2,1)          ; 
                       inf_th * Mij * Rj(:,2) ]; % thj not in cartesian measurements
     
    JC(rowInd,id1c) = [-inf_cart * eye(2) ; 
                        zeros(2,2)]; 
    JC(rowInd,id2c) = [ inf_cart * eye(2) ; 
                        zeros(2,2)]; 
  end
  %% current cost
  cost = norm(fhat)^2;
  
  %% delete anchor
  Jtot = [JC(:,3:end) JR(:,2:end)]; % cartesian part first
  corrEst = - (Jtot'*Jtot) \ (Jtot' * fhat);% minus because of fhat
  
  %% update estimate
  poseEst(nonAnchorInds) = poseEst(nonAnchorInds) + corrEst; % update estimate, preserving anchors 
  
  %% check stopping condition
  relCostChange = abs((cost - costPrec)/costPrec);
  if (isDebug>0) 
    fprintf('Current cost: %f, norm of the correction: %f, relative decrease: %.10f \n', cost, norm(corrEst), relCostChange); 
  end   
  if norm(corrEst) < 1e-4 || relCostChange < 1e-5
    break;
  end
end

if (isDebug==2) 
  figure
  plot(poseEst(1:2:2*nrNodes),poseEst(2:2:2*nrNodes),'-k');
  title('Estimate from refinePose2D')
end

end