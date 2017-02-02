
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