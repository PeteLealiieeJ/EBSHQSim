function [Xdot] = quadOdeFunctionHF(t,X,eaVec,distVec,P)
% quadOdeFunctionHF : Ordinary differential equation function that models
%                     quadrotor dynamics -- high-fidelity version.  For use
%                     with one of Matlab's ODE solvers (e.g., ode45).
%
%
% INPUTS
%
% t ---------- Scalar time input, as required by Matlab's ODE function
%              format.
%
% X ---------- Nx-by-1 quad state, arranged as 
%
%              X = [rI',vI',RBI(1,1),RBI(2,1),...,RBI(2,3),RBI(3,3),...
%                   omegaB',omegavec']'
%
%              rI = 3x1 position vector in I in meters
%              vI = 3x1 velocity vector wrt I and in I, in meters/sec
%             RBI = 3x3 attitude matrix from I to B frame
%          omegaB = 3x1 angular rate vector of body wrt I, expressed in B
%                   in rad/sec
%          omegavec = 4x1 vector of rotor angular rates, in rad/sec.
%                   omegavec(i) is the angular rate of the ith rotor.
%
%    eaVec --- 4x1 vector of voltages applied to motors, in volts.  eaVec(i)
%              is the constant voltage setpoint for the ith rotor.
%
%  distVec --- 3x1 vector of constant disturbance forces acting on the quad's
%              center of mass, expressed in Newtons in I.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% Xdot ------- Nx-by-1 time derivative of the input vector X
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Pete Lealiiee
%+==============================================================================+
g = P.constants.g;
m = P.quadParams.m;
rotor_loc = P.quadParams.rotor_loc;
Jq = P.quadParams.Jq;
omegaRdir = P.quadParams.omegaRdir;
kN = P.quadParams.kN;
kF = P.quadParams.kF;


wBX = crossProductEquivalent(X(16:18));
    % skew-symmetric matrix of angular rate of body used for cross product
    % including the variable 
    
RBI = reshape(X(7:15), [3,3]);
    % building the direction Cosine Matrix from the 9x1 description of
    % individual elements within the 18x1 vector description of state
    % within the X vector


NB = zeros(3,1);
Fsumvec = zeros(3,1);
    % The size of these vectors are known and previous states of the
    % initial vector are used to do a += operation within the for-loop so
    % their size is pre-built and elements within these vectors are set to
    % zero

for i=1:4
    NB = NB + ( crossProductEquivalent(rotor_loc(:,i)) * [ 0; 0; kF(i) * X(18+i)^2]) ...
        + [0; 0; -kN(i) * omegaRdir(i) * X(18+i)^2];
    Fsumvec = Fsumvec + [0; 0; kF(i) * X(18+i)^2];
end
    % Calculating NB from Sum of angular Momentum Terms 

Xdot = zeros(22,1);
    %The size of the output matrix is known so it is prebuilt before adding
    %setting values at indexed positions in order to not extend the size of
    %the matrix everytie something is added 
    
Xdot(1:3) = X(4:6);
    % Kinematic Relation
Xdot(4:6) = ( -(1/(2*m)) * P.quadParams.Cd * P.quadParams.Ad * P.constants.rho...
    * abs( (RBI'*[0; 0; 1])' * X(4:6) ) * (X(4:6)) ) + [0; 0; -g] + ((1/m) * RBI' * Fsumvec) + (distVec/m);
    % Center of Mass Theorum and sum of Forces including aerdynamic forces
Xdot(7:15) = - wBX * RBI;
    % Time derivative of the Euler Formula 
Xdot(16:18) = (Jq\( NB - (wBX * (Jq * X(16:18)))));
    % Time derivative of the body angular rate
Xdot(19:22) = ( 1./P.quadParams.taum(:) ) .* ( (P.quadParams.cm(:) .* eaVec) - X(19:22) );
    % time derivative of the rotor angular rate
    
end

