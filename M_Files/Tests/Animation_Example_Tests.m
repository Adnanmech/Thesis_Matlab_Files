% inertial.m: Animates trajectories for 5 cases demonstrating
%             inertial motion.  Here are the cases:
%             1. basic case:  wind stress in x and y directions  cyan
%                 of .1 Nm^-2, f=1x10^-4 s^-1, linear friction,
%                 mixed layer depth h=20m, no initial velocity.
%             2. Same as case 1, but no friction.                yellow
%             3. No wind forcing.  Water is given an initial     green
%                velocity of 10 cm/s. 
%             4. Wind is turned off after 2 days.                red 
%             5. Mixed layer depth is doubled (h=40m).           blue    

% Load all the variables which have been 'saved' as a .mat file:
load inert2.mat

% Set the starting point (x1,y1) (same as (0,0)) for each trajectory:
hstart1=line(x1(1),y1(1),'color','c','erase','none','linewidth',3);
hstart2=line(xr0(1),yr0(1),'color','y','erase','none');
hstart3=line(xnowind(1),ynowind(1),'color','g','erase','none');
hstart4=line(xrelax(1),yrelax(1),'color','r','erase','none');
hstart5=line(xdeep(1),ydeep(1),'color','b','erase','none');

% label axes, etc.
axis([-1000 25000 -25000 1000]); axis('equal') 
xlabel('x (m)')
ylabel('y (m)')
title('Inertial Motion Trajectories') 

% Loop through each trajectory, changing the data points.
for k=1:length(xdeep),
  set(hstart1,'xdata',x1(k),'ydata',y1(k));
  set(hstart2,'xdata',xr0(k),'ydata',yr0(k));
  set(hstart3,'xdata',xnowind(k),'ydata',ynowind(k));
  set(hstart4,'xdata',xrelax(k),'ydata',yrelax(k));
  set(hstart5,'xdata',xdeep(k),'ydata',ydeep(k));
  drawnow
end
axis([-1000 25000 -25000 1000]); axis('equal')