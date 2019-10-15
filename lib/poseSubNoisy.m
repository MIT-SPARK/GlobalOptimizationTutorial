function [deltaPoseNoisy] = poseSubNoisy(p_j , p_i, sigmaT, sigmaR, isUniform)
% pose_sub = (p_j , p_i) = inv(p_i) * p_j + noise
% returns the planar roto-translation that tranform pj in pi
% plus noise (sigmaT is the std of the noise on the cartesian components,
% sigmaR is the std of the noise added to the angle before wrapping)

if nargin < 5
   isUniform = 0; % Gaussian noise by default 
end

if nargin == 2
  sigmaT = 0;
  sigmaR = 0;
end

p_j = p_j(:);
p_i = p_i(:);

rho_i = p_i(1:2);
rho_j = p_j(1:2);
th_i = p_i(3);
th_j = p_j(3);

R = rot2D(th_i);

delta_th = wrapToPi(th_j - th_i);

if(abs(delta_th) > pi)
    disp('Error in pose_sub')
    disp(delta_th)
end

deltaPose = [ R' * (rho_j   -  rho_i)
    delta_th  ];

if isUniform == 0 % Gaussian noise
    noise = [sigmaT*randn(2,1) ;  sigmaR*randn];
else % uniform noise in the interval [-sigma, + sigma]
    noise = [sigmaT*2*(rand(2,1)-0.5*ones(2,1)) ; sigmaR*2*(rand-0.5)];
end
    
deltaPoseNoisy = deltaPose + noise;
deltaPoseNoisy(3) = wrapToPi(deltaPoseNoisy(3)); % wrap angle
