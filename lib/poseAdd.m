function [sum_pose] = poseAdd(p_j , p_i)   
% pose_add = (p_j , p_i)  the second is the delta pose to be added
% returns the planar roto-translation that compose pj and pi

if length(p_j(:,1))==1
    p_j = p_j';
end

if length(p_i(:,1))==1
    p_i = p_i';
end

rho_i = p_i(1:2);
rho_j = p_j(1:2);
th_i = p_i(3);
th_j = p_j(3);

R = rot2D(th_j);

sum_th = wrapToPi(th_j + th_i);

if(abs(sum_th) > pi)
    disp('Error in pose_sub')
   disp(sum_th) 
end

sum_pose = [ (rho_j + R * rho_i)   
                  sum_th  ];
