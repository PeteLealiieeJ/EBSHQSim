%% Top-level script for calling simulateQuadrotorEstimationAndControl
% author: Pete Lealiiee & Alireza Pedram

% 'clear all' is needed to clear out persistent variables from run to run
clear all; clc;
% Seed Matlab's random number: this allows you to simulate with the same noise
% every time (by setting a nonnegative integer seed as argument to rng) or
% simulate with a different noise realization every time (by setting
% 'shuffle' as argument to rng).
rng(10);

% Determines whether event-based control is used. Can be enabled to check
% whether something is wrong with the quads execution of the trajectory in
% a sense that neglects the use of EBC

S.EBC = true;

% Obstacle parameters as described in the visualizeQuad.m source code where
% slMat is the Mx3 Matrix pertaining to the respective xyz side lengths of
% M objects in the simulation enviornment. Defined early in order to
% implement proximity sensor trigger when covaince ellipse approaches
% obstacle margin [TO BE IMPLEMENTED]

scale = 0.1;
Obs.slMat = [ 20,20,40; ...
          20,20,40; ...
          20,20,40] * scale;
      
Obs.ogMat = [ 20,60,-20; ...
          50,50,-20; ...
          20,20,-20] * scale;    

%%
      
load("ref_traj_alpha_02")

% If 'smooth' is true, then the input trajectory is smoothed with
% subsequent derivatives. If false, trajctory is intrepolated to
% 0.005 second waypint time seperation and resuting velocities are
% constant between given waypoints by dxn/dt and desired accelerations are
% zero vectors.
smooth = true;
[t_ext,x_ext, v_ext, a_ext, P_ext] = iadt1p(t_ext,x_ext,P_ext, smooth);


%%

% Populate reference trajectory
% R.tVec = tVec;
R.tVec = t_ext;
N = length(R.tVec);

% R.rIstar = [r*cos(n*tVec),r*sin(n*tVec),ones(N,1)];
R.rIstar = x_ext' ;


% R.vIstar = [-r*n*sin(n*tVec),r*n*cos(n*tVec),zeros(N,1)];
R.vIstar = v_ext';


% R.aIstar = [-r*n*n*cos(n*tVec),-r*n*n*sin(n*tVec),zeros(N,1)];
R.aIstar = a_ext';


% The desired xI points toward the origin. The code below also normalizes
% each row in R.xIstar.
% R.xIstar = diag(1./vecnorm(R.rIstar'))*(-R.rIstar);
% R.xIstar = v_ext'/norm(v_ext) ;


% determine the orientation of quadrotor

%R.xIstar = ones(size(R.rIstar,1) , size(R.rIstar,2) ) .* (R.rIstar(end,:) - R.rIstar(1,:)) ;
 for jj=1: N-1
     
     dummy = (R.rIstar(jj+1,:) - R.rIstar(jj,:));
     R.xIstar(jj,:) = dummy/norm(dummy);
      %R.xIstar(jj,:)=[0 -1 0]; 
 end 
 

R.xIstar(N,:) = R.xIstar(N-1,:);



% Load the Covariance threshold in Matfile
R.Pthres = P_ext;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Matrix of disturbance forces acting on the body, in Newtons, expressed in I
S.distMat = 0*randn(N-1,3);
% Initial position in m
S.state0.r = R.rIstar(1,:)';
% Initial attitude expressed as Euler angles, in radians
S.state0.e = [0 0 asin(R.xIstar(1,2)) ]';
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [0 0 0]';
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = [0 0 0]';
% intial angular rates of rotors
S.state0.omegavec = [0 0 0 0]';
% Oversampling factor
S.oversampFact = 2;
% Feature locations in the I frame
% S.rXIMat = [0,0,0; 0,0,0.7]; 
S.rXIMat = [];
% Quadrotor parameters and constants
quadParamsScript;
constantsScript;
sensorParamsScript;
P.quadParams = quadParams; 
P.constants = constants; 
P.sensorParams = sensorParams;

[Q,CK] = simulateQuadrotorEstimationAndControl(R,S,P);

num_meas = sum(CK.meas_made);

%% Visualizer: GIF
% 
% S2.tVec = Q.tVec;
% S2.rMat = Q.state.rMat;
% S2.eMat = Q.state.eMat;
% S2.Pser = CK.Pser;
% S2.Posblf = CK.Posblf;
% S2.Pthres = R.Pthres;
% S2.Posidl = R.rIstar;
% S2.tspan = R.tVec;
% S2.plotFrequency = 5; % was 20
% S2.makeGifFlag = true;
% S2.videoname = 'Ali_sim_2.avi';
% %S2.gifFileName = 'alpha_2.gif';
% S2.bounds=[0 10 0 10 -10 10];
% %visualizeQuad(S2,Obs);
% visQuad(S2,Obs);
%% Visualizer: Static shots

S2.tVec = Q.tVec;
S2.rMat = Q.state.rMat;
S2.eMat = Q.state.eMat;
S2.Pser = CK.Pser;
S2.Posblf = CK.Posblf;
S2.Pthres = R.Pthres;
S2.Posidl = R.rIstar;
S2.tspan = R.tVec;
S2.plotFrequency = 0.8; % was 20
S2.makeGifFlag = true;
S2.gifFileName = 'EBCvis.gif';
S2.bounds=[0 10 0 10 -10 10];

staticVisualizeQuad(S2,Obs,Q);

S3 = S2;
S3.plotFrequency = 2;
S3.makeGifFlag = false;
S3.gifFileName = 'EBCCovvis.gif';
staticVisualizeCOV(S3,Obs,Q);
%% Static Plots/Figures
n = 3;
figure(n);clf;
n = n + 1;
hold on
plot(R.tVec,abs(CK.Poserr(1,:)))
plot(R.tVec,abs(CK.Poserr(2,:)))
plot(R.tVec,abs(CK.Poserr(3,:)))
hold off
legend("X Belief Error","Y Belief Error","Z Belief Error")
xlabel("time [s] ")
ylabel("Belief Error (Difference Between where it believes it is and where it actually is) [m]")

figure(n);clf;
n = n + 1;
%figure('WindowState','maximized')
plot(Q.tVec,Q.state.rMat(:,3), 'LineWidth',2 ); grid on;

    set(gca, 'FontName', 'Arial', 'FontSize', 22)
    set(gca,'color','white');
    grid on
    set(gca,'LineWidth',1)
    ax = gca;
    ax.LineWidth = 1;

axis([0 15 -1.0 0.5])
xlabel('Time (sec)');
ylabel('Z (m)');

pbaspect([4 1 1])
%title('Vertical position of CM'); 

% % % figure(n);clf;
% % % n = n + 1;
% % % psiError = unwrap(n*Q.tVec + pi - Q.state.eMat(:,3));
% % % meanPsiErrorInt = round(mean(psiError)/2/pi);
% % % plot(Q.tVec,psiError - meanPsiErrorInt*2*pi);
% % % grid on;
% % % xlabel('Time (sec)');
% % % ylabel('\Delta \psi (rad)');
% % % title('Yaw angle error');
% % 
% % figure(n);clf;
% % n = n + 1;
% % plot( x_ext(1,:),x_ext(2,:) )
% % hold on
% % plot(Q.state.rMat(:,1), Q.state.rMat(:,2));
% % hold off
% % axis equal; grid on;
% % xlabel('X (m)');
% % ylabel('Y (m)');
% % legend('Ideal Trajectory', 'Quad Trajectory')
% % title('Horizontal position of CM');
% % 
% % figure(n);clf;
% % n = n + 1;
% % plot3( R.rIstar(:,1),R.rIstar(:,2),R.rIstar(:,3)  )
% % hold on
% % plot3(Q.state.rMat(:,1), Q.state.rMat(:,2), Q.state.rMat(:,3));
% % for jj = 1:size(Obs.slMat,1)
% %     hold on 
% %     plotcube(Obs.slMat(jj,:),Obs.ogMat(jj,:),.8,[1 0 0]);
% % end
% % hold off
% % axis equal; grid on;
% % xlabel('X (m)');
% % ylabel('Y (m)');
% % zlabel('Z (m)');
% % legend('Ideal Trajectory', 'Quad Trajectory')
% % title('3-Dimensional position of CM');

%% save data 

% % len= length(Q.tVec);
% % t_2=zeros(1,len);
% % z_2=zeros(1,len);
% % 
% % for ii=1:len
% % t_2(ii)=Q.tVec(ii);
% % z_2(ii)= Q.state.rMat(ii,3);
% % end
% % 
% % save('z_data_2.mat','t_2','z_2');