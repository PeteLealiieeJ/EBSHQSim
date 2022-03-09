function [t_out,x_out,v_out,a_out,P_out] = iadt1p(t_ext,x_ext,P_ext, smooth)
    % iadt1p : Returns interpolated simulation data with 0.005 time
    %          spacing nessesary for accurate simulation usage (Note that 
    %           this was moved from the topsimulate.m file to be more 
    %           readable so it looks "clunky" )
    %
    %
    % INPUTS
    %
    % t_ext ---------- 1xn time vector
    %
    % x_ext ---------- Either 3xn or 2xn column-stacked position matrix
    %
    % P_ext ---------- Either 3x3xn or 2x2xn Cov Matrix Stack
    %
    % smooth --------- boolean telling function whether or not to return a
    %                  9th order polynomial smoothed trajectory
    %
    %
    % OUTPUTS
    %
    % t_out ---------- 1 x m time vector with 0.005 spacing from 0
    %
    % x_out ---------- Either 3xm or 2xm interpolated column-stacked 
    %                  position matrix
    %
    % v_out ---------- Either 3xm or 2xm interpolated column-stacked 
    %                  position matrix
    %
    % a_out ---------- Either 3xm or 2xm interpolated column-stacked 
    %                  position matrix
    %
    % P_out ---------- Either 3x3xm or 2x2xm interpolated Cov Matrix stack
    %
    %
    %
    %+------------------------------------------------------------------------------+
    %
    %
    % Author:  Pete Lealiiee
    %+==============================================================================+
    [dod,~] = size(x_ext);
    t_ext_cpy = t_ext; 
    t_ext = 0:0.005:round(t_ext_cpy(end),2)+0.005;
    t_out = t_ext;
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

            P11_ext2d = interp1(t_ext_cpy,P11_ext2d',t_ext)';
            P12_ext2d = interp1(t_ext_cpy,P12_ext2d',t_ext)';
            P21_ext2d = interp1(t_ext_cpy,P21_ext2d',t_ext)';
            P22_ext2d = interp1(t_ext_cpy,P22_ext2d',t_ext)';


            P_out = zeros(2,2,length(P11_ext2d));

            for ii = 1:length(P11_ext2d)
                P_out(1,1,ii) = P11_ext2d(ii);
                P_out(1,2,ii) = P12_ext2d(ii);
                P_out(2,1,ii) = P21_ext2d(ii);
                P_out(2,2,ii) = P22_ext2d(ii);
            end


            if(~smooth)
                x_intp1 = interp1(t_ext_cpy,x_ext',t_ext)';
                dt = t_ext(2)-t_ext(1);

                xv = (x_intp1(1,1:end-1) - x_intp1(1,2:end))/dt;
                yv = (x_intp1(2,1:end-1) - x_intp1(2,2:end))/dt;
                zv = zeros(1,length(xv));

                x_out = [x_intp1; zeros(1,length(x_intp1)) ];
                v_out = [xv , 0 ; yv , 0 ; zv , 0 ];
                a_out = zeros(3,length(v_out));
            else
                xPF = polyfit(t_ext_cpy,x_ext(1,:),9);
                xvPF = polyder(xPF);
                xaPF = polyder(xvPF);

                yPF = polyfit(t_ext_cpy,x_ext(2,:),9);
                yvPF = polyder(yPF);
                yaPF = polyder(yvPF);


                tspan = 0:0.005:t_ext(end);

                x_out = [ polyval(xPF,tspan) ; polyval(yPF,tspan) ; zeros(1,length(tspan))  ];
                v_out = [ polyval(xvPF,tspan) ; polyval(yvPF,tspan) ; zeros(1,length(tspan))  ];
                a_out = [ polyval(xaPF,tspan) ; polyval(yaPF,tspan) ; zeros(1,length(tspan))  ];
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


           P11_ext2d = interp1(t_ext_cpy,P11_ext2d',t_ext)';
           P12_ext2d = interp1(t_ext_cpy,P12_ext2d',t_ext)';
           P13_ext2d = interp1(t_ext_cpy,P13_ext2d',t_ext)';
           P21_ext2d = interp1(t_ext_cpy,P21_ext2d',t_ext)';
           P22_ext2d = interp1(t_ext_cpy,P22_ext2d',t_ext)';
           P23_ext2d = interp1(t_ext_cpy,P23_ext2d',t_ext)';
           P31_ext2d = interp1(t_ext_cpy,P31_ext2d',t_ext)';
           P32_ext2d = interp1(t_ext_cpy,P32_ext2d',t_ext)';
           P33_ext2d = interp1(t_ext_cpy,P33_ext2d',t_ext)'; 

            P_out = zeros(3,3,length(P11_ext2d));

            for ii = 1:length(P11_ext2d)
                P_out(1,1,ii) = P11_ext2d(ii);
                P_out(1,2,ii) = P12_ext2d(ii);
                P_out(1,3,ii) = P13_ext2d(ii);
                P_out(2,1,ii) = P21_ext2d(ii);
                P_out(2,2,ii) = P22_ext2d(ii);
                P_out(2,3,ii) = P23_ext2d(ii);
                P_out(3,1,ii) = P31_ext2d(ii);
                P_out(3,2,ii) = P32_ext2d(ii);
                P_out(3,3,ii) = P33_ext2d(ii);
            end


            if(~smooth)
                x_intp1 = interp1(t_ext_cpy,x_ext_cpy',t_ext)';
                dt = t_ext(2)-t_ext(1);

                xv = (x_intp1(1,1:end-1) - x_intp1(1,2:end))/dt;
                yv = (x_intp1(2,1:end-1) - x_intp1(2,2:end))/dt;
                zv = (x_intp1(3,1:end-1) - x_intp1(3,2:end))/dt;
                
                x_out = x_intp1;
                v_out = [xv , 0 ; yv , 0 ; zv , 0 ];
                a_out = zeros(3,length(v_out));
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

                x_out = [ polyval(xPF,tspan) ; polyval(yPF,tspan) ;  polyval(zPF,tspan)  ];
                v_out = [ polyval(xvPF,tspan) ; polyval(yvPF,tspan) ; polyval(zvPF,tspan) ];
                a_out = [ polyval(xaPF,tspan) ; polyval(yaPF,tspan) ;  polyval(zaPF,tspan)  ];
            end

        otherwise
            error('Error: Loaded Trajectory must either be in Inertial 2-D or 3-D Frame.')
    end
    %--------------------------------------------------------------------------
end