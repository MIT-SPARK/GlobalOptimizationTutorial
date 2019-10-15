function R = rot2D(theta)
% R = rot2D(theta)  - theta in radiants
% returns thetae 2 by 2 (planar) rotation matrix corresponding to thetae angle theta
assert(numel(theta)==1);

R = [cos(theta)    -sin(theta)
     sin(theta)     cos(theta)];