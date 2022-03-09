function P = visQuad(S,Obs)
% author: Alireza Pedram

Vid_traj = VideoWriter(S.videoname);
Vid_traj.FrameRate = 30;
Vid_traj.Quality = 80;


open(Vid_traj);



%% Videos specs
time_pos = [8.5, 9.7];
x_range = [0, 10];
y_range = [0, 10];
fig_path = figure;
figure(fig_path)

color_jet = jet(100);

sim_step= length(S.tVec);
%sime_step=100;

T=0.0025;

%% obstacle specs
obstacle_edge = obstacle_lab();
edges = [obstacle_edge(:).start];
edges_re = reshape(edges,[2,numel(edges)/2]);

% You need to designate the number of vertices of each obstacle here
% The first element should be 0
num_vert = [0, 4, 4, 4];

% Color used to fill the obstacles
%color_fill = [1, 0, 0]; 
color_fill = [187/255,139/255,172/255];
color_b = [0    0.4470    0.7410];
% traget region
target = [8, 9 ; 0, 1]; 

%% Quad params
%figureNumber = 1; figure(figureNumber); clf;
%fcounter = 0; %frame counter for gif maker
%m = length(S.tVec);

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


%% Interpolation must be used to smooth timing
% Create time vectors
    tf = 0.0025;
    tmax = S.tVec(sim_step); 
    tmin = S.tVec(1);
    tPlot = tmin:tf:tmax;
    tPlotLen = length(tPlot);
    
    % Interpolate to regularize times
    [t2unique, indUnique] = unique(S.tVec);
    rPlot = (interp1(t2unique, S.rMat(indUnique,:), tPlot))';
    ePlot = (interp1(t2unique, S.eMat(indUnique,:), tPlot))';
    
    Ps11 = zeros(1,length(S.Pser));
    Ps12 = zeros(1,length(S.Pser));
    Ps21 = zeros(1,length(S.Pser));
    Ps22 = zeros(1,length(S.Pser));
    
    Pt11 = zeros(1,length(S.Pthres));
    Pt12 = zeros(1,length(S.Pthres));
    Pt21 = zeros(1,length(S.Pthres));
    Pt22 = zeros(1,length(S.Pthres));

    for ii = 1:length(S.Pser)
       Ps11(ii) = S.Pser(1,1,ii);
       Ps12(ii) = S.Pser(1,2,ii);
       Ps21(ii) = S.Pser(2,1,ii);
       Ps22(ii) = S.Pser(2,2,ii);
    end
    
    
    for jj = 1:length(S.Pthres)
       Pt11(jj) = S.Pthres(1,1,jj);
       Pt12(jj) = S.Pthres(1,2,jj);
       Pt21(jj) = S.Pthres(2,1,jj);
       Pt22(jj) = S.Pthres(2,2,jj);
    end
    
    %disp(length(S.rMat))
    
    Ps11Plot = (interp1(S.tspan, Ps11(:), tPlot))';
    Ps12Plot = (interp1(S.tspan, Ps12(:), tPlot))';
    Ps21Plot = (interp1(S.tspan, Ps21(:), tPlot))';
    Ps22Plot = (interp1(S.tspan, Ps22(:), tPlot))';
    
    Pt11Plot = (interp1(S.tspan, Pt11(:), tPlot))';
    Pt12Plot = (interp1(S.tspan, Pt12(:), tPlot))';
    Pt21Plot = (interp1(S.tspan, Pt21(:), tPlot))';
    Pt22Plot = (interp1(S.tspan, Pt22(:), tPlot))';
    
    PosblfPlot = (interp1(S.tspan,S.Posblf', tPlot))';
    PosidlPlot = (interp1(S.tspan,S.Posidl, tPlot))';

%% plot each frame

for i = 1:sim_step
    figure_setting(fig_path, x_range, y_range, time_pos, i, T)

% plot obstacles
        ini = 0;
        hold on
        for kk = 1:length(num_vert)-1
            ini = ini + num_vert(kk);
            las = ini + num_vert(kk+1);
            fill( edges_re(1, ini+1:las).' , edges_re(2, ini+1:las).' ,color_fill)
        end
        
        for kk = 1:length(obstacle_edge)
            fill([obstacle_edge(kk).start(1) obstacle_edge(kk).end(1)], [obstacle_edge(kk).start(2) obstacle_edge(kk).end(2)],'k','LineWidth',0.5)
        end
        
     % plot the boundry of target region
     
     x_patch = [target(1,1) target(1,2) target(1,2) target(1,1)];
     y_patch = [target(2,1) target(2,1) target(2,2) target(2,2)];
     z_patch= [-1 -1 -1.1 -1.1];
     patch(x_patch,y_patch, z_patch, [0.8 1 0.8])
     
     rectangle('Position',[target(1,1) target(2,1) target(1,2)-target(1,1) target(2,2)-target(2,1)],'EdgeColor',[0 0.5 0],'LineWidth',1.5)
     
 %% plot data    
     % Extract data
        RIB = euler2dcm(ePlot(1:3,i))';
        
        r = rPlot(1:3,i);
        Posblf = PosblfPlot(1:3,i);
        Posidl = PosidlPlot(1:3,i);
        Ps = [Ps11Plot(i) , Ps12Plot(i) ; Ps21Plot(i) , Ps22Plot(i)];
        Pt = [Pt11Plot(i) , Pt12Plot(i) ; Pt21Plot(i) , Pt22Plot(i)];
        
        

        
       % Plot quad itself  
         
        body_scale = 2;
        % Translate, rotate, and plot the rotors
        hold on
        view(0,90)
        rotor1_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,1)*ones(1,20)) *  body_scale ;
        rotor1plot = plot3(rotor1_circle(1,:), rotor1_circle(2,:),...
            rotor1_circle(3,:), 'Color',rgbOrange);
        hold on
        rotor2_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,2)*ones(1,20)) *  body_scale ;
        rotor2plot = plot3(rotor2_circle(1,:), rotor2_circle(2,:),...
            rotor2_circle(3,:), 'Color',rgbOrange);
        hold on
        rotor3_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,3)*ones(1,20)) *  body_scale ;
        rotor3plot = plot3(rotor3_circle(1,:), rotor3_circle(2,:),...
            rotor3_circle(3,:), 'black');
        hold on
        rotor4_circle=r*ones(1,20)+RIB*(circpts(:,:)+rotorLocations(:,4)*ones(1,20)) *  body_scale ;
        rotor4plot = plot3(rotor4_circle(1,:), rotor4_circle(2,:),...
            rotor4_circle(3,:), 'black');
        
        % Translate, rotate, and plot the body
        RIB = RIB *  body_scale; 
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
        axis2 = quiver3(r(1),r(2),r(3),bodyY(1),bodyY(2),bodyY(3),'blue');
        hold on
        axis3 = quiver3(r(1),r(2),r(3),bodyZ(1),bodyZ(2),bodyZ(3),'green');
     

     %**Plot Covariance Threshold
        [a,b,phi,~] = error_ellipse([Posidl(1),Posidl(2),Posidl(3)]',Pt,chi);
    
         ellipse_x_r  = a * cos( theta_grid );
         ellipse_y_r  = b * sin( theta_grid );
         Rot = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
         r_ellipse = [ellipse_x_r;ellipse_y_r]' * Rot;
         hold on
         elpst_plt = plot3(r_ellipse(:,1) + Posidl(1)  , r_ellipse(:,2) + Posidl(2), zeros(length(r_ellipse)) ,'r-','LineWidth',1);
         threspt = plot3(Posidl(1),Posidl(2),Posidl(3)+1,'ro', 'MarkerSize',3,'MarkerFaceColor','r');        
        
      %**Plot 90p Confidence Area
        [a,b,phi,~] = error_ellipse([Posblf(1),Posblf(2),Posblf(3)]',Ps,chi);
    
         ellipse_x_r  = a * cos( theta_grid );
         ellipse_y_r  = b * sin( theta_grid );
         Rot = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
         r_ellipse = [ellipse_x_r;ellipse_y_r]' * Rot;
         hold on
         elpss_plt = plot3(r_ellipse(:,1) + Posblf(1)  , r_ellipse(:,2) + Posblf(2), zeros(length(r_ellipse)) ,'-','color',color_b ,'LineWidth',1);
         npctpt = plot3(Posblf(1),Posblf(2),Posblf(3)+2,'o','color',color_b,'MarkerSize',3,'MarkerFaceColor',color_b);
        
    frame = getframe(gcf);
    writeVideo(Vid_traj,frame); 
end

close(Vid_traj)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Setting of figure %%%%%%%%%%%%%%%%%%%%%%%
function figure_setting(fig_path, x_range, y_range, time_pos, i, T)

clf(fig_path);
set(fig_path,'color','white');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 16)
xlabel("X [m]", "Fontsize", 16);
ylabel("Y [m]", "Fontsize", 16);
hold on
grid on
box on
axis equal
set(gca,'LineWidth',1)
ax = gca;
ax.LineWidth = 1;
fig_path.Position = [100 100 500 460];

xlim(x_range);
ylim(y_range);

box on
hold on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

text(time_pos(1), time_pos(2), [num2str(round(T*i,2)), ' s'],'FontSize',12);
end


function r_ellipse = prep_ellipses_line(xhat_el, Phat_el, chi)
    
plot_th_grid = linspace(0, 2*pi, 100);

[ra_Rb, rb_Rb, ang_Rb, ~] = error_ellipse(xhat_el(1:2), Phat_el(1:2,1:2), chi);

ellipse_x_r  = ra_Rb * cos( plot_th_grid );
ellipse_y_r  = rb_Rb * sin( plot_th_grid );
Rot = [ cos( ang_Rb ) sin( ang_Rb ); -sin( ang_Rb ) cos( ang_Rb ) ];
r_ellipse = [ellipse_x_r;ellipse_y_r]' * Rot;
end

end

