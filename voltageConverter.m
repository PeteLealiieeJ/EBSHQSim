function [eak] = voltageConverter(Fk,NBk,P)
% voltageConverter : Generates output voltages appropriate for desired
%                    torque and thrust.
%
%
% INPUTS
%
% Fk --------- Commanded total thrust at time tk, in Newtons.
%
% NBk -------- Commanded 3x1 torque expressed in the body frame at time tk, in
%              N-m.
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
% eak -------- Commanded 4x1 voltage vector to be applied at time tk, in
%              volts. eak(i) is the voltage for the ith motor.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Pete Lealiiee
%+==============================================================================+  

alpha = 1;
beta = 0.9;
Fmax = P.quadParams.kF(1) * (P.quadParams.cm(1)^2) * (P.quadParams.eamax^2); 
Yvec = [ min(Fk,4*beta*Fmax) ; (alpha*NBk) ];

Gmat = [            1                                1                                  1                               1;... 
         P.quadParams.rotor_loc(2,1)      P.quadParams.rotor_loc(2,2)       P.quadParams.rotor_loc(2,3)     P.quadParams.rotor_loc(2,4);...
        -P.quadParams.rotor_loc(1,1)      -P.quadParams.rotor_loc(1,2)      -P.quadParams.rotor_loc(1,3)    -P.quadParams.rotor_loc(1,4);...
     -P.quadParams.kN(1)/P.quadParams.kF(1)     P.quadParams.kN(2)/P.quadParams.kF(2)        -P.quadParams.kN(3)/P.quadParams.kF(3)        P.quadParams.kN(4)/P.quadParams.kF(4) ];

Fvec = Gmat\Yvec;         
         
while max(Fvec) > Fmax
    alpha = alpha - 0.01;
    Yvec = [ min(Fk,4*beta*Fmax) ; (alpha*NBk) ];
    Fvec = Gmat\Yvec; 
end

for i = 1:4
  if Fvec(i) < 0
      Fvec(i) = 0;
  end
end

omegass = (Fvec ./ P.quadParams.kF).^(1/2);

eak = omegass ./ P.quadParams.cm;

end
  

