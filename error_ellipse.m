function [a,b,phi,ellipse_rect] = error_ellipse(x,P,chi)
% author: Alireza Pedram
[eigenvec, eigenval] = eig(P, 'vector');
% Get the index of the largest eigenvector
[max_evl, max_evc_ind_c] = max(eigenval);
max_evc = eigenvec(:, max_evc_ind_c);

% Get the smallest eigenvector and eigenvalue
[min_evl, ~] = min(eigenval);
% min_evc = eigenvec(:,min_evc_ind_c);

% Calculate the angle between the x-axis and the largest eigenvector
angle = atan2(max_evc(2),max_evc(1));

% This angle is between -pi and pi.
% Let's shift it such that the angle is between 0 and 2pi
if(angle < 0)
    angle = angle + 2*pi;
end

% Get the confidence interval error ellipse which probability is determined
% by "chi"
chisquare_val = sqrt(chi);
phi = angle;
X0  = x(1);
Y0  = x(2);
a   = chisquare_val*sqrt(max_evl);
b   = chisquare_val*sqrt(min_evl);

x_box = sqrt(a^2*cos(phi)^2 + b^2*sin(phi)^2);
y_box = sqrt(a^2*sin(phi)^2 + b^2*cos(phi)^2);
% rectangle around ellipse [bot_left-x bot_left-y w h]
ellipse_rect = [X0-x_box, Y0-y_box, 2*x_box, 2*y_box];