%Testing script
clear;
clc;
clear all;

%Diagonalized Matrix
% xx = zeros(10,10);
% 
% for kk = 1:10
% xx(kk,kk) = 15
% end


%Size of u
% u  = -1:.001:1;
% size(u)

%creates a 10x10 matrix of constants (5)
%    col , row     const
%mat1(1:10,1:10) = 5; 

%u  = -1:.001:1;
%rx1(1:2001,1) = 15;

%comb = rx1*u;


%% Parameters
g = 9.8;            %   Gravity acceleration [m/s^2]
m  = 1500;          %   Mass of the vehicle [kg]
Lf = 1.5;           %   Distance from front axle to CoG [m]
Lr = 1.5;           %   Distance from rear axle to CoG [m]
Lw = 1.5;           %   Distance between wheels [m]
hg = 0.4;           %   Hight of CoG [m]

Jz = 1/12*m*((Lf+Lr)^2+Lw^2);    %   Body moment of inertia around vertical axle
Jw = .6;            %   Wheel rotational moment of inertial >>Inertia = Mass(at radius r) * radius^2; sum multiple masses at diff radii for total
Rw = 0.5;           %   Wheel rolling radius [m]

% Magic formular (Longitudinal)
Kxnorm = 30;  %21.6;    % normalized stiffness (Raised to 70 to have better grip at low slips)
Muxp = .99;         %.8 peak friction coefficient
Muxs = 0.5;         % sliding friction coefficient was .5 kds 5/16/14

% Magic formular (Lateral)
Ky0 = 90000;        % static cornering stiffness [N/rad]
Fz0 = 3000;         % static vertical load     [N]
Fz = m*g/4;
Muyp0 = 0.2;       % static peak friction coefficient
Muys = 0.18;        % friction coefficient when sliding 

% Combined slip parameters
rx1 = 15;
rx2 = 15;
ry1 = 15;
ry2 = 15;

%Testing for AWD CONTROL

%Creates Slip angle vs Fxc at diff slip ratios


%{
%initial variables
u = 0.0;  


for k = 1:100
   %u  = u + .001;
   %KDS 5/30/14 - Try using for loop and stepping uu w/ constant u instead
   %of current set up
   uu = -.5:.001:.5;
   
   Bxc = rx1 * cos( atan (rx2 * u));
   %Bxct = transpose(Bxc);
   %Gxc = cos( atan (Bxct * uu ));
   Gxc = cos( atan (Bxc * uu ));
   
%
   Dx = Muxp;
   Cx = 2 - 2/pi * asin( Muxs/Muxp );
   Bx = Kxnorm / ( Cx*Dx );
   sp = 3 / ( Bx*Cx );   % slip ratio at which Mu is max.
   Ex = ( Bx * sp - tan( pi / ( 2 * Cx )) ) / ( Bx * sp-atan( Bx * sp ) );
   
   
   Fx0 = Dx*sin(Cx*atan(Bx*u-Ex*(Bx*u-atan(Bx*u))));
   Fx00(k) = Fx0;
   
   Fxc = Gxc * Fx0;
   Fxcplot{k} = Fxc;
   %plot Fxc vs uu
   plot(uu, Fxc)
   hold on
end

%}


%%Creates plot for slip ratio vs Fxc at various slip angles

uu = 0.0;

for k = 1:10
   uu  = uu + .05;
   u = -10:.001:10;
   
   Bxc = rx1 * cos( atan (rx2 * u));
   %Bxct = transpose(Bxc);
   %Gxc = cos( atan (Bxct * uu ));
   Gxc = cos( atan (Bxc * uu ));
   
%
   Dx = Muxp;
   Cx = 2 - 2/pi * asin( Muxs/Muxp );
   Bx = Kxnorm / ( Cx*Dx );
   sp = 3 / ( Bx*Cx );   % slip ratio at which Mu is max.
   Ex = ( Bx * sp - tan( pi / ( 2 * Cx )) ) / ( Bx * sp-atan( Bx * sp ) );
   
   
   Fx0 = Dx*sin(Cx*atan(Bx*u-Ex*(Bx*u-atan(Bx*u))));
   
   %Fxc = Gxc * Fx0;
   for gg = 1:20001
    Fxc(1,gg) = Gxc(1,gg)*Fx0(1,gg);
   end
   plot(u, Fxc)
   hold on
end

%{
%% plots for Fy
    u = 0.1;

   for k = 1:50

    %u  = u + .01;
    uu = -.5:.001:.5;
    sizeuu = size(uu);
    

    Byc = ry1 * cos( atan (ry2 * uu));
    Gyc = cos( atan (Byc * u));
    
    
    Ky = Ky0 * sin( 2 * atan(Fz / Fz0) );
    p = ( ( Muyp0 / Muys ) - 1 ) * Fz0;
    Muyp = ( Fz + p) * Muys / Fz;

    Kynorm = Ky / Fz;

    Dy = Muyp;
    Cy = 2 - 2 / pi * asin( Muys / Muyp );
    By = Kynorm / ( Cy * Dy );
    ap = 3 / ( By * Cy );  
    Ey = ( By * ap - tan( pi / ( 2 * Cy )))/( By * ap - atan( By * ap ));
    
    Fy0 = (Dy*sin(Cy*atan(By*uu-Ey*(By*uu-atan(By*uu)))));
   
    %Fyc = Gyc * Fy0;        %Fy0 is the pure lateral force
    for gg = 1:sizeuu(2)
      Fyc(1,gg) = Gyc(1,gg)*Fy0(1,gg);
    end
    plot(uu, Fyc,'g')
    hold on
    
   end
%}  
   
   %subplot(1, 2, 1);
  % for k = 1:100
  %   plot(Fyc,Fxcplot{k})
  % end 

%{
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%Plots Fx vs Fy at fixed slip angles w varying slip ratio
%initial variables
%u = -0.04;
uu = 0;


   %u  = u + .001;
   %KDS 5/30/14 - Try using for loop and stepping uu w/ constant u instead
   %of current set up
   u = -1:.001:1;
   
   Bxc = rx1 * cos( atan (rx2 * u));
   %Bxct = transpose(Bxc);
   %Gxc = cos( atan (Bxct * uu ));
   Gxc = cos( atan (Bxc * uu ));
   
%
   Dx = Muxp;
   Cx = 2 - 2/pi * asin( Muxs/Muxp );
   Bx = Kxnorm / ( Cx*Dx );
   sp = 3 / ( Bx*Cx );   % slip ratio at which Mu is max.
   Ex = ( Bx * sp - tan( pi / ( 2 * Cx )) ) / ( Bx * sp-atan( Bx * sp ) );
   
   
   Fx0 = Dx*sin(Cx*atan(Bx*u-Ex*(Bx*u-atan(Bx*u))));
   %Fx00(k) = Fx0;
 for k = 1:2001  
   Fxc(k) = Gxc(k) * Fx0(k);
   %Fxcplot{k} = Fxc;
   %plot(uu, Fxc)
   hold on
end   
 

%% plots for Fy
    %u = 0.2;

   for k = 1:100

    %u  = u + .001;
    %uu = -.5:.001:.5;
    u = -1:.001:1;

    Byc = ry1 * cos( atan (ry2 * uu));
    Gyc = cos( atan (Byc * u));
    
    
    Ky = Ky0 * sin( 2 * atan(Fz / Fz0) );
    p = ( ( Muyp0 / Muys ) - 1 ) * Fz0;
    Muyp = ( Fz + p) * Muys / Fz;

    Kynorm = Ky / Fz;

    Dy = Muyp;
    Cy = 2 - 2 / pi * asin( Muys / Muyp );
    By = Kynorm / ( Cy * Dy );
    ap = 3 / ( By * Cy );  
    Ey = ( By * ap - tan( pi / ( 2 * Cy )))/( By * ap - atan( By * ap ));
    
    Fy0 = (Dy*sin(Cy*atan(By*uu-Ey*(By*uu-atan(By*uu)))));
   
    Fyc = Gyc * Fy0;        %Fy0 is the pure lateral force
    
    %plot(uu, Fyc,'g')
    hold on
    
   end
  
   
   %subplot(1, 2, 2);
   for k = 1:100
     plot(Fyc,Fxc)
   end 

%}