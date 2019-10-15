function rotError = get_rot_error(R_est,R_gt,d)

nrNodes = size(R_est,1) / d;
rotError = zeros(nrNodes, 1);
for i=1:nrNodes
    Rdiff = R_gt(blkIndices(i,d),:)' * R_est(blkIndices(i,d),:); 
    rotError(i, 1) = abs( atan2(Rdiff(2, 1), Rdiff(1, 1)) );
end