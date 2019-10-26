function R_tilde = build_data_matrix(edges,nrNodes,d)
% given edge measurements, nrNodes and d, build the matrix R_tilde

m = size(edges,1); % nr of measurements
R_tilde = zeros(d*nrNodes,d*nrNodes);
for k=1:m
    i = edges(k,1);
    j = edges(k,2);
    Rij_tilde = rot2D(edges(k,5));
    R_tilde(blkIndices(i,2),blkIndices(j,2)) = -Rij_tilde';
    R_tilde(blkIndices(j,2),blkIndices(i,2)) = -Rij_tilde;
end

if norm(R_tilde - R_tilde', Inf) > 1e-6
   error('Q is not symmetric') 
end
