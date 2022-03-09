function [R_BW] = euler2dcm(e)
% euler2dcm : Converts Euler angles phi = e(1), theta = e(2), and psi = e(3)
%             (in radians) into a direction cosine matrix for a 3-1-2 rotation.
%
% Let the world (W) and body (B) reference frames be initially aligned.  In a
% 3-1-2 order, rotate B away from W by angles psi (yaw, about the body Z
% axis), phi (roll, about the body X axis), and theta (pitch, about the body Y
% axis).  R_BW can then be used to cast a vector expressed in W coordinates as
% a vector in B coordinates: vB = R_BW * vW
%
% INPUTS
%
% e ---------- 3-by-1 vector containing the Euler angles in radians: phi =
%              e(1), theta = e(2), and psi = e(3)
%
%
% OUTPUTS
%
% R_BW ------- 3-by-3 direction cosine matrix 
% 
%+------------------------------------------------------------------------------+
% References: NA
%
%
% Author: Pete Lealiiee Jr 
%+==============================================================================+  

% %     3-1-2 Euler Rotation with the rotation matrix formula
%     R_BW = rotationMatrix([0;1;0], e(2)) * rotationMatrix([1;0;0], e(1)) * rotationMatrix([0;0;1], e(3));
%      
    phi = e(1); theta = e(2); psi = e(3);
    
    R_BW = [ (cos(theta)*cos(psi)) - (sin(theta)*sin(phi)*sin(psi)) , (cos(theta)*sin(psi)) + (sin(theta)*sin(phi)*cos(psi)) , -(cos(phi)*sin(theta)); ... 
                          -(cos(phi)*sin(psi))                      ,                  cos(phi)*cos(psi)                     ,        sin(phi)  ; ...
             (sin(theta)*cos(psi)) + (cos(theta)*sin(phi)*sin(psi)) , ( sin(theta)*sin(psi) ) - ( cos(theta)*sin(phi)*cos(psi) ) ,    cos(theta)*cos(phi)     ];
    
end
  








