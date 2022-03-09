function [rpGtilde,rbGtilde] = gnssMeasSimulator(S,P)
% gnssMeasSimulator : Simulates GNSS measurements for quad.
%
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position of CM in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
%
% OUTPUTS
%
% rpGtilde --- 3x1 GNSS-measured position of the quad's primary GNSS antenna,
%              in ECEF coordinates relative to the reference antenna, in
%              meters.
%
% rbGtilde --- 3x1 GNSS-measured position of secondary GNSS antenna, in ECEF
%              coordinates relative to the primary antenna, in meters.
%              rbGtilde is constrained to satisfy norm(rbGtilde) = b, where b
%              is the known baseline distance between the two antennas.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Pete Lealiiee
%+==============================================================================+  

% GNSS position measurement of primary antenna
RLG = Recef2enu(P.sensorParams.r0G);
RP = RLG' *  P.sensorParams.RpL * RLG ;

rPI = S.statek.rI + ( S.statek.RBI' * P.sensorParams.rA(:,1) );
rPG = RLG' * rPI;

wPG = mvnrnd(zeros(3,1), RP)';

rpGtilde = rPG + wPG;

% GNSS position measurement of secondary antenna

rSI = S.statek.rI + ( S.statek.RBI' * P.sensorParams.rA(:,2) );
rSG = RLG' * rSI;

rC = rSG - rPG;
rCnorm = norm(rC);
rCunit = rC/rCnorm;

eta = 1e-8;

RC = (rCnorm^2 * P.sensorParams.sigmab^2 *...
    ( eye(3,3) - (rCunit * rCunit') )) + (eta * eye(3,3));

wCG = mvnrnd(zeros(3,1), RC)';

rCtilde_temp = rC + wCG;

rbGtilde = (rCtilde_temp/norm(rCtilde_temp)) * rCnorm;
end





