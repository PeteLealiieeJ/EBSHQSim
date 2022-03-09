%% Top-level script for calling simulateQuadrotorEstimationAndControl
clear all;clc;
first_trail = 1;
last_trail = 100;
%sim_step=3001; % for alpha=2.0 
%sim_step=2601; % for alpha=0.2

num_meas_vec = zeros(1,last_trail-first_trail+1);

for iji = first_trail:last_trail

    % 'clear all' is needed to clear out persistent variables from run to run
    clc
    fprintf('Current Trail: %d \n',iji);    
    % Seed Matlab's random number: this allows you to simulate with the same noise
    % every time (by setting a nonnegative integer seed as argument to rng) or
    % simulate with a different noise realization every time (by setting
    % 'shuffle' as argument to rng).
    rng(iji);

    % Determines whether event-based control is used. Can be enabled to check
    % whether something is wrong with the quads execution of the trajectory in
    % a sense that neglects the use of EBC

    S.EBC = true;


    load("ref_traj_alpha_02")

    % If 'smooth' is true, then the input trajectory is smoothed with
    % subsequent derivatives. If false, trajctory is intrepolated to
    % 0.005 second waypint time seperation and resuting velocities are
    % constant between given waypoints by dxn/dt and desired accelerations are
    % zero vectors.
    smooth = true;


    % Below filters through and interpolates between points to match required 
    % time step for simulator
    %--------------------------------------------------------------------------
    [dod,~] = size(x_ext);
    t_ext_cpy = 0:1:length(t_ext)-1; 
    t_ext = 0:0.005:t_ext_cpy(end);

    switch(dod)

        case(2)

            P11_ext2d = zeros(1,length(P_ext));
            P12_ext2d = zeros(1,length(P_ext));
            P21_ext2d = zeros(1,length(P_ext));
            P22_ext2d = zeros(1,length(P_ext));

            for ii = 1:length(P_ext)
               P11_ext2d(ii) = P_ext(1,1,ii);
               P12_ext2d(ii) = P_ext(1,2,ii);
               P21_ext2d(ii) = P_ext(2,1,ii);
               P22_ext2d(ii) = P_ext(2,2,ii);

            end

    %         t_ext = 0:1:length(t_ext)-1;
    %
    %         for ii = 1:(length(t_ext)-1)
    % 
    % 
    %             if ii==1
    %                 kk = ii;
    %             else
    %                 kk = kk + jj - 1;
    %             end
    % 
    %             tsec = t_ext(kk):0.005:t_ext(kk+1);   
    %             t_ext = [ t_ext(1:kk) , tsec(2:end-1) , t_ext(kk+1:end) ];

                if(~smooth)
    %                 xsec = linspace( x_ext(1,kk),x_ext(1,kk+1),length(tsec));
    %                 ysec = linspace( x_ext(2,kk),x_ext(2,kk+1),length(tsec));
    %                 x_ext = [ x_ext(1,1:kk) , xsec(2:end-1) , x_ext(1,kk+1:end) ;...
    %                       x_ext(2,1:kk) , ysec(2:end-1) , x_ext(2,kk+1:end)];

                    x_ext = interp1(t_ext_cpy,x_ext',t_ext)';

                end

    %             P11sec = linspace( P11_ext2d(kk),P11_ext2d(kk+1),length(tsec));
    %             P12sec = linspace( P12_ext2d(kk),P12_ext2d(kk+1),length(tsec));
    %             P21sec = linspace( P21_ext2d(kk),P21_ext2d(kk+1),length(tsec));
    %             P22sec = linspace( P22_ext2d(kk),P22_ext2d(kk+1),length(tsec));
    % 
    % 
    %             P11_ext2d = [  P11_ext2d(1:kk) , P11sec(2:end-1) , P11_ext2d(kk+1:end) ];    
    %             P12_ext2d = [  P12_ext2d(1:kk) , P12sec(2:end-1) , P12_ext2d(kk+1:end) ];    
    %             P21_ext2d = [  P21_ext2d(1:kk) , P21sec(2:end-1) , P21_ext2d(kk+1:end) ];    
    %             P22_ext2d = [  P22_ext2d(1:kk) , P22sec(2:end-1) , P22_ext2d(kk+1:end) ];    

                P11_ext2d = interp1(t_ext_cpy,P11_ext2d',t_ext)';
                P12_ext2d = interp1(t_ext_cpy,P12_ext2d',t_ext)';
                P21_ext2d = interp1(t_ext_cpy,P21_ext2d',t_ext)';
                P22_ext2d = interp1(t_ext_cpy,P22_ext2d',t_ext)';


    %             jj = length(tsec);
    % 
    %         end

            P_ext = zeros(2,2,length(P11_ext2d));

            for ii = 1:length(P11_ext2d)
                P_ext(1,1,ii) = P11_ext2d(ii);
                P_ext(1,2,ii) = P12_ext2d(ii);
                P_ext(2,1,ii) = P21_ext2d(ii);
                P_ext(2,2,ii) = P22_ext2d(ii);
            end


            if(~smooth)
                dt = t_ext(2)-t_ext(1);

                xv = (x_ext(1,1:end-1) - x_ext(1,2:end))/dt;
                yv = (x_ext(2,1:end-1) - x_ext(2,2:end))/dt;
                zv = zeros(1,length(xv));

                x_ext = [x_ext; zeros(1,length(x_ext)) ];
                v_ext = [xv , 0 ; yv , 0 ; zv , 0 ];
                a_ext = zeros(3,length(v_ext));
            else
                xPF = polyfit(t_ext_cpy,x_ext(1,:),9);
                xvPF = polyder(xPF);
                xaPF = polyder(xvPF);

                yPF = polyfit(t_ext_cpy,x_ext(2,:),9);
                yvPF = polyder(yPF);
                yaPF = polyder(yvPF);


                tspan = 0:0.005:t_ext(end);

                x_ext = [ polyval(xPF,tspan) ; polyval(yPF,tspan) ; zeros(1,length(tspan))  ];
                v_ext = [ polyval(xvPF,tspan) ; polyval(yvPF,tspan) ; zeros(1,length(tspan))  ];
                a_ext = [ polyval(xaPF,tspan) ; polyval(yaPF,tspan) ; zeros(1,length(tspan))  ];
            end

        case(3)

            P11_ext2d = zeros(1,length(P_ext));
            P12_ext2d = zeros(1,length(P_ext));
            P13_ext2d = zeros(1,length(P_ext));
            P21_ext2d = zeros(1,length(P_ext));  
            P22_ext2d = zeros(1,length(P_ext));
            P23_ext2d = zeros(1,length(P_ext));
            P31_ext2d = zeros(1,length(P_ext));
            P32_ext2d = zeros(1,length(P_ext));
            P33_ext2d = zeros(1,length(P_ext));

            for ii = 1:length(P_ext)
               P11_ext2d(ii) = P_ext(1,1,ii);
               P12_ext2d(ii) = P_ext(1,2,ii);
               P13_ext2d(ii) = P_ext(1,3,ii);
               P21_ext2d(ii) = P_ext(2,1,ii);
               P22_ext2d(ii) = P_ext(2,2,ii);
               P23_ext2d(ii) = P_ext(2,3,ii);
               P31_ext2d(ii) = P_ext(3,1,ii);
               P32_ext2d(ii) = P_ext(3,2,ii);
               P33_ext2d(ii) = P_ext(3,3,ii);

            end

    %         for ii = 1:(length(t_ext)-1)
    % 
    % 
    %             if ii==1
    %                 kk = ii;
    %             else
    %                 kk = kk + jj - 1;
    %             end
    % 
    %             tsec = t_ext(kk):0.005:t_ext(kk+1);   
    %             t_ext = [ t_ext(1:kk) , tsec(2:end-1) , t_ext(kk+1:end) ];

                if(~smooth)
    %                 xsec = linspace( x_ext(1,kk),x_ext(1,kk+1),length(tsec));
    %                 ysec = linspace( x_ext(2,kk),x_ext(2,kk+1),length(tsec));
    %                 zsec = linspace( x_ext(3,kk),x_ext(3,kk+1),length(tsec));
    % 
    %                 x_ext = [ x_ext(1,1:kk) , xsec(2:end-1) , x_ext(1,kk+1:end) ;...
    %                       x_ext(2,1:kk) , ysec(2:end-1) , x_ext(2,kk+1:end); ...
    %                       x_ext(3,1:kk) , zsec(2:end-1) , x_ext(3,kk+1:end)];
    %                 
                    x_ext = interp1(t_ext_cpy,x_ext_cpy',t_ext)';

                end

    %             P11sec = linspace( P11_ext2d(kk),P11_ext2d(kk+1),length(tsec));
    %             P12sec = linspace( P12_ext2d(kk),P12_ext2d(kk+1),length(tsec));
    %             P13sec = linspace( P13_ext2d(kk),P13_ext2d(kk+1),length(tsec));
    %             P21sec = linspace( P21_ext2d(kk),P21_ext2d(kk+1),length(tsec));
    %             P22sec = linspace( P22_ext2d(kk),P22_ext2d(kk+1),length(tsec));
    %             P23sec = linspace( P23_ext2d(kk),P23_ext2d(kk+1),length(tsec));
    %             P31sec = linspace( P31_ext2d(kk),P31_ext2d(kk+1),length(tsec));
    %             P32sec = linspace( P32_ext2d(kk),P32_ext2d(kk+1),length(tsec));
    %             P33sec = linspace( P33_ext2d(kk),P33_ext2d(kk+1),length(tsec));
    % 
    % 
    % 
    %             P11_ext2d = [  P11_ext2d(1:kk) , P11sec(2:end-1) , P11_ext2d(kk+1:end) ];    
    %             P12_ext2d = [  P12_ext2d(1:kk) , P12sec(2:end-1) , P12_ext2d(kk+1:end) ]; 
    %             P13_ext2d = [  P13_ext2d(1:kk) , P13sec(2:end-1) , P13_ext2d(kk+1:end) ];
    %             P21_ext2d = [  P21_ext2d(1:kk) , P21sec(2:end-1) , P21_ext2d(kk+1:end) ];    
    %             P22_ext2d = [  P22_ext2d(1:kk) , P22sec(2:end-1) , P22_ext2d(kk+1:end) ];   
    %             P23_ext2d = [  P23_ext2d(1:kk) , P23sec(2:end-1) , P23_ext2d(kk+1:end) ];
    %             P31_ext2d = [  P31_ext2d(1:kk) , P31sec(2:end-1) , P31_ext2d(kk+1:end) ];    
    %             P32_ext2d = [  P32_ext2d(1:kk) , P32sec(2:end-1) , P32_ext2d(kk+1:end) ];   
    %             P33_ext2d = [  P33_ext2d(1:kk) , P33sec(2:end-1) , P33_ext2d(kk+1:end) ];
    % 
    %             jj = length(tsec);
    % 
    %         end

           P11_ext2d = interp1(t_ext_cpy,P11_ext2d',t_ext)';
           P12_ext2d = interp1(t_ext_cpy,P12_ext2d',t_ext)';
           P13_ext2d = interp1(t_ext_cpy,P13_ext2d',t_ext)';
           P21_ext2d = interp1(t_ext_cpy,P21_ext2d',t_ext)';
           P22_ext2d = interp1(t_ext_cpy,P22_ext2d',t_ext)';
           P23_ext2d = interp1(t_ext_cpy,P23_ext2d',t_ext)';
           P31_ext2d = interp1(t_ext_cpy,P31_ext2d',t_ext)';
           P32_ext2d = interp1(t_ext_cpy,P32_ext2d',t_ext)';
           P33_ext2d = interp1(t_ext_cpy,P33_ext2d',t_ext)'; 

            P_ext = zeros(3,3,length(P11_ext2d));

            for ii = 1:length(P11_ext2d)
                P_ext(1,1,ii) = P11_ext2d(ii);
                P_ext(1,2,ii) = P12_ext2d(ii);
                P_ext(1,3,ii) = P13_ext2d(ii);
                P_ext(2,1,ii) = P21_ext2d(ii);
                P_ext(2,2,ii) = P22_ext2d(ii);
                P_ext(2,3,ii) = P23_ext2d(ii);
                P_ext(3,1,ii) = P31_ext2d(ii);
                P_ext(3,2,ii) = P32_ext2d(ii);
                P_ext(3,3,ii) = P33_ext2d(ii);
            end


            if(~smooth)
                dt = t_ext(2)-t_ext(1);

                xv = (x_ext(1,1:end-1) - x_ext(1,2:end))/dt;
                yv = (x_ext(2,1:end-1) - x_ext(2,2:end))/dt;
                zv = (x_ext(3,1:end-1) - x_ext(3,2:end))/dt;

                v_ext = [xv , 0 ; yv , 0 ; zv , 0 ];
                a_ext = zeros(3,length(v_ext));
            else
                xPF = polyfit(t_ext_cpy,x_ext(1,:),9);
                xvPF = polyder(xPF);
                xaPF = polyder(xvPF);

                yPF = polyfit(t_ext_cpy,x_ext(2,:),9);
                yvPF = polyder(yPF);
                yaPF = polyder(yvPF);

                zPF = polyfit(t_ext_cpy,x_ext(3,:),9);
                zPF = polyder(zPF);
                zaPF = polyder(zvPF);

                tspan = 0:0.005:124;

                x_ext = [ polyval(xPF,tspan) ; polyval(yPF,tspan) ;  polyval(zPF,tspan)  ];
                v_ext = [ polyval(xvPF,tspan) ; polyval(yvPF,tspan) ; polyval(zvPF,tspan) ];
                a_ext = [ polyval(xaPF,tspan) ; polyval(yaPF,tspan) ;  polyval(zaPF,tspan)  ];
            end

        otherwise
            error('Error: Loaded Trajectory must either be in Inertial 2-D or 3-D Frame.')
    end
    %--------------------------------------------------------------------------


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
    S.state0.e = [0 0 R.xIstar(1,2) ]';
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
    num_meas_vec(iji) = num_meas;
    %num_meas_vec(:,iji)=CK.meas_made;
    
    
    clearvars -except num_meas_vec ;
    clear imuSimulator;
    clear stateEstimatorUKF;
end


