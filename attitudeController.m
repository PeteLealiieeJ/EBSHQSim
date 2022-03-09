function [NBk] = attitudeController(R,S,P)
% attitudeController : Controls quadcopter toward a reference attitude
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%       zIstark = 3x1 desired body z-axis direction at time tk, expressed as a
%                 unit vector in the I frame.
%
%       xIstark = 3x1 desired body x-axis direction, expressed as a
%                 unit vector in the I frame.
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude
%
%                  vI = 3x1 velocity with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%
% OUTPUTS
%
% NBk -------- Commanded 3x1 torque expressed in the body frame at time tk, in
%              N-m.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Pete Lealiiee 
%+==============================================================================+  

K = 1.3*diag([0.8 0.8 0.2]); Kd = 0.7*diag([0.4 0.4 0.1]);

RBI = S.statek.RBI;

z_cross = crossProductEquivalent(R.zIstark);

b_cross = crossProductEquivalent( (z_cross...
    * R.xIstark  )/ sqrt((z_cross * R.xIstark)'*...
   (z_cross * R.xIstark)));

RBIstark_trans = [ b_cross *  R.zIstark,...
  (z_cross * R.xIstark )/sqrt((z_cross * R.xIstark)'* (z_cross * R.xIstark)),...
  R.zIstark ];

RBIstark = RBIstark_trans';

RE = RBIstark * RBI';

eE = [ RE(2,3) - RE(3,2) ; RE(3,1) - RE(1,3) ; RE(1,2) - RE(2,1) ];

omegaB_cross = crossProductEquivalent(S.statek.omegaB);

NBk = ( K * eE ) - ( Kd * S.statek.omegaB  ) + ( omegaB_cross * P.quadParams.Jq *  S.statek.omegaB  );

end





  

