function [e] = dcm2euler(R_BW)
% dcm2euler : Converts a direction cosine matrix R_BW to Euler angles phi =
%             e(1), theta = e(2), and psi = e(3) (in radians) for a 3-1-2
%             rotation. If the conversion to Euler angles is singular (not
%             unique), then this function issues an error instead of returning
%             e.
%
% Let the world (W) and body (B) reference frames be initially aligned.  In a
% 3-1-2 order, rotate B away from W by angles psi (yaw, about the body Z
% axis), phi (roll, about the body X axis), and theta (pitch, about the body Y
% axis).  R_BW can then be used to cast a vector expressed in W coordinates as
% a vector in B coordinates: vB = R_BW * vW
%
% INPUTS
%
% R_BW ------- 3-by-3 direction cosine matrix 
%
%
% OUTPUTS
%
% e ---------- 3-by-1 vector containing the Euler angles in radians: phi =
%              e(1), theta = e(2), and psi = e(3).  By convention, these
%              should be constrained to the following ranges: -pi/2 <= phi <=
%              pi/2, -pi <= theta < pi, -pi <= psi < pi.  
% 
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Pete Lealiiee Jr
%+==============================================================================+  
  
    if ~((R_BW(2,3) == 1 || R_BW(2,3) == -1))
        
        % asin of value gives in desired range -pi/2 <= x < pi/2 
        phi = asin(R_BW(2,3));
        
        
        theta = atan2(-R_BW(1,3),R_BW(3,3));
        if theta == pi
            theta = -pi;
        end
        
        psi = atan2(-R_BW(2,1),R_BW(2,2));
        if psi == pi
            psi = -pi;
        end
        
    else
        error('dcm2euler:incorrectInputType',...
           'Error. \nAdditional Input Required the R_BW(2,3) value equals %d, R_BW(3,1) equals %d and R_BW(1,2) equal %d so solving method tends to infinity with asin(0/0).', R_BW(3,2), R_BW(3,1),R_BW(1,2));
    end

    e = [phi ; theta ; psi];
end







