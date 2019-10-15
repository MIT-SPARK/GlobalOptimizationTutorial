function R = project2SO2(M)

if size(M,1)~=2 || size(M,2)~=2
   error('project2SO2 can only take 2x2 matrices as input') 
end

% R = [cos -sin, sin cos]
[U,S,V] = svd(M);
R = U*V';

if det(R)<0
   warning('det(R)<0 (check projection to SO2)') 
   R = [1 0; 0 -1]*R;
end
