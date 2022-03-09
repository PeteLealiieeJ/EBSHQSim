
function [] = staticVisualizeQuad(S,Obs,Q)

% visualizeQuad : Takes in an input structure S, Obs and Q, visualizes the resulting
%                 2D motion with instentaneous shots of the quad its
%                 covariance and its covariance threshold
%
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%           rMat = 3xM matrix of quad positions, in meters
%
%           eMat = 3xM matrix of quad attitudes, in radians
%
%           tVec = Mx1 vector of times corresponding to each measurement in
%                  rMat
%
%           tspan = Nx1 vector of times corresponding to each measurement in
%                  Posidl
%
%  plotFrequency = The scalar number of frames of the plot per each second of
%                  input data.  Expressed in Hz.
%
%         bounds = 6x1, the 3d axis size vector
%
%    makeGifFlag = Boolean (if true, export the current plot to a .gif)
%
%    gifFileName = A string with the file name of the .gif if one is to be
%                  created.  Make sure to include the .gif exentsion.
%
%        Pthres = 2x2xN Pthres(:,:,k) is the covariance matrix 
%                 threshold at time tk = tVec(k), which if
%                 exceeded,causes the simulation to use the       
%                 simulated positioning GNSS measurement in estimating
%                 the quads position at time tVec(k+1)
%  
%          Pser = 2x2xN covariance matrix estimate series. Pser(:,:,k) is
%                 the estimated covariance matrix at times tk = tVec(k),
%  
%        Posblf = Nx3 matrix composed such that Posblf(k,:)' is the 3x1
%                 position the quad believes it is at (but most likely is not at),
%                 in meters.
%
%        Posidl = Nx3 matrix of desired CM positions in the I frame, in
%                 meters.  rIstar(k,:)' is the 3x1 position at time tk =
%                 tVec(k).
%
% **O ---------- Structure with the following elements ( N Rect. Obstacles):
%
%           **slMat = Nx3 matrix of Cube Side Lengths, in meters
%
%           **ogMat = Nx3 matrix of Cube Front Left Edge, in meters (looking down x-axis)
%
%
% OUTPUTS
%
% P ---------- Structure with the following elements:
%
%          tPlot = Nx1 vector of time points used in the plot, sampled based
%                  on the frequency of plotFrequency
%
%          rPlot = 3xN vector of positions used to generate the plot, in
%                  meters.
%
%          ePlot = 3xN vector of attitudes used to generate the plot, in
%                  radians.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Inspiration from:  Nick Montalbano (Radionavigation Lab)
% Author: Pete Lealiiee
%+==============================================================================+
  

    figureNumber = 2; figure(figureNumber); clf;

    % Important params
    m = length(S.tVec);

    chi = chi2inv(0.90,2);
    theta_grid = linspace(0,2*pi,20);


    % RBG scaled on [0,1] for the color orange
    rgbOrange=[1 .4 0];

    % Parameters for the rotors
    rotorLocations=[0.105 0.105 -0.105 -0.105
        0.105 -0.105 0.105 -0.105
        0 0 0 0];
    r_rotor = .062;

    % Determines the location of the corners of the body box in the body frame,
    % in meters
    bpts=[ 120  120 -120 -120  120  120 -120 -120
        28  -28   28  -28   28  -28   28  -28
        20   20   20   20   -30   -30   -30   -30 ]*1e-3;
    % Rectangles representing each side of the body box
    b1 = [bpts(:,1) bpts(:,5) bpts(:,6) bpts(:,2) ];
    b2 = [bpts(:,1) bpts(:,5) bpts(:,7) bpts(:,3) ];
    b3 = [bpts(:,3) bpts(:,7) bpts(:,8) bpts(:,4) ];
    b4 = [bpts(:,1) bpts(:,3) bpts(:,4) bpts(:,2) ];
    b5 = [bpts(:,5) bpts(:,7) bpts(:,8) bpts(:,6) ];
    b6 = [bpts(:,2) bpts(:,6) bpts(:,8) bpts(:,4) ];

    % Create a circle for each rotor
    t_circ=linspace(0,2*pi,20);
    circpts=zeros(3,20);
    for i=1:20
        circpts(:,i)=r_rotor*[cos(t_circ(i));sin(t_circ(i));0];
    end


    % Create time vectors
        tf = 1/S.plotFrequency;
        tmax = S.tVec(m); tmin = S.tVec(1);
        tPlot = tmin:tf:tmax;
        tPlotLen = length(tPlot);

        % Interpolate to regularize times
        [t2unique, indUnique] = unique(S.tVec);
        rPlot = (interp1(t2unique, S.rMat(indUnique,:), tPlot))';
        ePlot = (interp1(t2unique, S.eMat(indUnique,:), tPlot))';

        Ps11 = zeros(1,length(S.Pser));
        Ps12 = zeros(1,length(S.Pser));
        PS1 = zeros(1,length(S.Pser));
        PS2 = zeros(1,length(S.Pser));

        Pt11 = zeros(1,length(S.Pthres));
        Pt12 = zeros(1,length(S.Pthres));
        Pt21 = zeros(1,length(S.Pthres));
        Pt22 = zeros(1,length(S.Pthres));

        for ii = 1:length(S.Pser)
           Ps11(ii) = S.Pser(1,1,ii);
           Ps12(ii) = S.Pser(1,2,ii);
           PS1(ii) = S.Pser(2,1,ii);
           PS2(ii) = S.Pser(2,2,ii);
        end


        for jj = 1:length(S.Pthres)
           Pt11(jj) = S.Pthres(1,1,jj);
           Pt12(jj) = S.Pthres(1,2,jj);
           Pt21(jj) = S.Pthres(2,1,jj);
           Pt22(jj) = S.Pthres(2,2,jj);
        end


        Ps11Plot = (interp1(S.tspan, Ps11(:), tPlot))';
        Ps12Plot = (interp1(S.tspan, Ps12(:), tPlot))';
        PS1Plot = (interp1(S.tspan, PS1(:), tPlot))';
        PS2Plot = (interp1(S.tspan, PS2(:), tPlot))';

        Pt11Plot = (interp1(S.tspan, Pt11(:), tPlot))';
        Pt12Plot = (interp1(S.tspan, Pt12(:), tPlot))';
        Pt21Plot = (interp1(S.tspan, Pt21(:), tPlot))';
        Pt22Plot = (interp1(S.tspan, Pt22(:), tPlot))';

        PosblfPlot = (interp1(S.tspan,S.Posblf', tPlot))';
        PosidlPlot = (interp1(S.tspan,S.Posidl, tPlot))';

        figure(figureNumber);
    %     figure('WindowState','maximized')
    %     pause(2)

        %Plot obstacles
        
        hold on
        plot( S.Posidl(:,1),S.Posidl(:,2) );
        hold on
        plot(Q.state.rMat(:,1), Q.state.rMat(:,2));

        
        for jj = 1:size(Obs.slMat,1)
            hold on 
            plotcube(Obs.slMat(jj,:),Obs.ogMat(jj,:),.8,[1 0 0]);
        end

        % Iterate through points
        for i=1:tPlotLen


            % Extract data
            RIB = euler2dcm(ePlot(1:3,i))';

            r = rPlot(1:3,i);
            Posblf = PosblfPlot(1:3,i);
            Posidl = PosidlPlot(1:3,i);
            Ps = [Ps11Plot(i) , Ps12Plot(i) ; PS1Plot(i) , PS2Plot(i)];
            Pt = [Pt11Plot(i) , Pt12Plot(i) ; Pt21Plot(i) , Pt22Plot(i)];


            %**Plot 90p Confidence Area
            [a,b,phi,~] = error_ellipse([Posblf(1),Posblf(2),Posblf(3)]',Ps,chi);

             ellipse_x_r  = a * cos( theta_grid );
             ellipse_y_r  = b * sin( theta_grid );
             Rot = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
             r_ellipse = [ellipse_x_r;ellipse_y_r]' * Rot;
             hold on
             elpss_plt = plot3(r_ellipse(:,1) + Posblf(1)  , r_ellipse(:,2) + Posblf(2), zeros(length(r_ellipse)) ,'g-','LineWidth',0.5);
             npctpt = plot3(Posblf(1),Posblf(2),Posblf(3),'g*');

            %**Plot Covariance Threshold
            [a,b,phi,~] = error_ellipse([Posidl(1),Posidl(2),Posidl(3)]',Pt,chi);

             ellipse_x_r  = a * cos( theta_grid );
             ellipse_y_r  = b * sin( theta_grid );
             Rot = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
             r_ellipse = [ellipse_x_r;ellipse_y_r]' * Rot;
             hold on
             elpst_plt = plot3(r_ellipse(:,1) + Posidl(1)  , r_ellipse(:,2) + Posidl(2), zeros(length(r_ellipse)) ,'b-','LineWidth',0.5);
             threspt = plot3(Posidl(1),Posidl(2),Posidl(3),'b*');

            % Translate, rotate, and plot the rotors
            hold on
            view(0,90)
            rotor1_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,1)*ones(1,20));
            rotor1plot = plot3(rotor1_circle(1,:), rotor1_circle(2,:),...
                rotor1_circle(3,:), 'Color',rgbOrange);
            hold on
            rotor2_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,2)*ones(1,20));
            rotor2plot = plot3(rotor2_circle(1,:), rotor2_circle(2,:),...
                rotor2_circle(3,:), 'Color',rgbOrange);
            hold on
            rotor3_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,3)*ones(1,20));
            rotor3plot = plot3(rotor3_circle(1,:), rotor3_circle(2,:),...
                rotor3_circle(3,:), 'black');
            hold on
            rotor4_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,4)*ones(1,20));
            rotor4plot = plot3(rotor4_circle(1,:), rotor4_circle(2,:),...
                rotor4_circle(3,:), 'black');

            % Translate, rotate, and plot the body
            b1r=r*ones(1,4)+RIB*b1; b2r=r*ones(1,4)+RIB*b2; b3r=r*ones(1,4)+RIB*b3;
            b4r=r*ones(1,4)+RIB*b4; b5r=r*ones(1,4)+RIB*b5; b6r=r*ones(1,4)+RIB*b6;
            X = [b1r(1,:)' b2r(1,:)' b3r(1,:)' b4r(1,:)' b5r(1,:)' b6r(1,:)'];
            Y = [b1r(2,:)' b2r(2,:)' b3r(2,:)' b4r(2,:)' b5r(2,:)' b6r(2,:)'];
            Z = [b1r(3,:)' b2r(3,:)' b3r(3,:)' b4r(3,:)' b5r(3,:)' b6r(3,:)'];
            hold on
            bodyplot=patch(X,Y,Z,[.5 .5 .5]);

            % Translate, rotate, and plot body axes
            bodyX=0.5*RIB*[1;0;0]; bodyY=0.5*RIB*[0;1;0]; bodyZ=0.5*RIB*[0;0;1];
            hold on
            axis1 = quiver3(r(1),r(2),r(3),bodyX(1),bodyX(2),bodyX(3),'red');
            hold on
            axiS = quiver3(r(1),r(2),r(3),bodyY(1),bodyY(2),bodyY(3),'blue');
            hold on
            axis3 = quiver3(r(1),r(2),r(3),bodyZ(1),bodyZ(2),bodyZ(3),'green');
            
        end

    axis(S.bounds); grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    legend('Ideal Trajectory', 'Quad Trajectory')
    title('Horizontal position of CM');

end