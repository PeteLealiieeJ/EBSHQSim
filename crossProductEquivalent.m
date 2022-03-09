function [uCross] = crossProductEquivalent(u)
% crossProductEquivalent : Outputs the cross-product-equivalent matrix uCross
%                          such that for arbitrary 3-by-1 vectors u and v,
%                          cross(u,v) = uCross*v.
%
% INPUTS
%
% u ---------- 3-by-1 vector
%
%
% OUTPUTS
%
% uCross ----- 3-by-3 skew-symmetric cross-product equivalent matrix
%
%+------------------------------------------------------------------------------+
% References: [1]"A Tutorial on Vector and Attitude" by Jozef C. van der Ha &
%              Malcolm D. Shuster
%
%
% Author: Pete Lealiiee Jr 
%+==============================================================================+
        

    % Checking Size of input vector to catch any errors with orientation, builds, etc. 
    if size(u,1) ~= 3 || size(u,2) ~=1
       error('crossProductEquivalent:incorrectInputType',...
           'Error. \nInput must be a 3x1 vector, not a %dx%d vector.',...
           size(u,1), size(u,2) )
    end
    
    
    %Making the Skew-Symettric Cross-Product Matrix [1]
    uCross = [ 0 -u(3) u(2) ; u(3) 0 -u(1) ; -u(2) u(1) 0 ];

end