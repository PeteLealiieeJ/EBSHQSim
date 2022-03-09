function [make_meas] = ellipse_escape (x, P, x_planned ,P_planned)
% author: Alireza Pedram
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
make_meas = false;

chi = chi2inv(0.90,2); % 90percent region chi2inv(0.90,2)

%chi=1;
[ra,rb,ang,~] = error_ellipse(x, P, chi);


for ii=0:100
    
theta = ii * 2 * pi/(99);

ellipse_x_r  = ra*cos(theta);
ellipse_y_r  = rb*sin( theta );
        
Rot = [ cos(ang) sin(ang); -sin(ang) cos(ang) ];
r_ellipse =  x' + [ellipse_x_r; ellipse_y_r]' * Rot;


dis=(x_planned'-r_ellipse)* P_planned^(-1) * (x_planned'-r_ellipse)';

if dis > chi
    make_meas = true;
    break
end

end

