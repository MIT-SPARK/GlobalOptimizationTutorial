function matrixIndices = blockToMatIndices(blockIndex, blockSize)
% We have a matrix with 2x2 blocks. We say that we want to access block
% i and the function returns (2*i-1:2*i), i.e., the entries that belong to
% the block

numBlocks = length(blockIndex);
matrixIndices = zeros(numBlocks*blockSize,1);

for i=1:numBlocks
  matrixIndices(blockSize*i-(blockSize-1) : blockSize*i) = [blockSize*blockIndex(i)-(blockSize-1) : blockSize*blockIndex(i)];
end