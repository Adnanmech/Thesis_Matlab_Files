%clc, close all, clear all

%% 1. set axis and figure

myaxes = axes('xlim',[-2 50],'ylim',[-10 10], 'zlim',[-2 2]);
torqueaxes = axes('xlim',[0 6],'ylim',[-150 150]);
subplot(myaxes);

%plot details
subplot(2,2,3)
xlabel('Time [s]')
ylabel('TFL (purple) TFR (blue) [Nm]')
subplot(2,2,4)
xlabel('Time [s]')
ylabel('TRL (red) TRR (teal) [Nm]')

main_vehicle_plot = subplot(2,2,1);
%torque_FL_plot    = subplot(2,2,2);

view(0,90);        % VIEW(3) sets the default 3-D view, AZ = -37.5, EL = 30
grid on;
axis equal;
hold on
xlabel('x')
ylabel('y')
zlabel('z')

%% 1.1 Create Road lines for reference
%l1=line([.5 .5],[0 1],'parent',a1
% for s = 0:50
%     p = s*3;
%     line([-5 5],[p p],'color',[0 0 1]);
% end

%% 2. generate the cylinder and the cone and the sphere

%Creation of Wheels for Vehicle
[xcylinder ycylinder zcylinder] = cylinder([0.3 0.3]);
zcylinder(2, :) = .5;
%[xcone ycone zcone] = cylinder([0.1 0.0]); %cone
%[xsphere ysphere zsphere] = sphere();
%[xbox ybox zbox] = 

%Bottom Layer of Car
p1 = [-.75 -1.5 0];
p2 = [.75 -1.5 0];
p3 = [.75 1.5 0];
p4 = [-.75 1.5 0]; 

x = [p1(1) p2(1) p3(1) p4(1)];
y = [p1(2) p2(2) p3(2) p4(2)];
z = [p1(3) p2(3) p3(3) p4(3)];

%Top Layer of Car
p1_1 = [-.75 -1.5 .3];
p2_1 = [.75 -1.5 .3];
p3_1 = [.75 1.5 .3];
p4_1 = [-.75 1.5 .3]; 

x1 = [p1_1(1) p2_1(1) p3_1(1) p4_1(1)];
y1 = [p1_1(2) p2_1(2) p3_1(2) p4_1(2)];
z1 = [p1_1(3) p2_1(3) p3_1(3) p4_1(3)];




%% 3. plot different orientations of these shapes
% assign all surfaces to a variable h
 % h positions:
 % h(1) = RR
 % h(2) = RL
 % h(3) = FL
 % h(4) = FR

%Plot Wheels

    h(1) = surface(.5 + zcylinder,-1.5+xcylinder,ycylinder);
    h(2) = surface(-.5 - zcylinder,-1.5+xcylinder,ycylinder);
    h(3) = surface(-.5 - zcylinder,1.5+xcylinder,ycylinder);
    h(4) = surface(.5 + zcylinder,1.5+xcylinder,ycylinder);
    
%Wheel Coloring
    %CData_RR = rand(size(get(h(1),'CData')));
    %CData_RL = rand(size(get(h(2),'CData')));
    %CData_FL = rand(size(get(h(3),'CData')));
    %CData_FR = rand(size(get(h(4),'CData')));
    %CData = get(h(1),'CData')
    
    for hh = 1:19
        CData_RR(1,hh) = -0.9;
        CData_RR(2,hh) = -0.9;
        
        CData_RL(1,hh) = -0.9;
        CData_RL(2,hh) = -0.9;
        
        CData_FL(1,hh) = -0.3;
        CData_FL(2,hh) = -0.3;
        
        CData_FR(1,hh) = -0.3;
        CData_FR(2,hh) = -0.3;
    end
    
    CData_RR(1,20) = 1;
    CData_RL(1,20) = 1;
    CData_FL(1,20) = 1;
    CData_FR(1,20) = 1;
    
    set(h(1),'CData',CData_RR);
    set(h(2),'CData',CData_RL);
    set(h(3),'CData',CData_FL);
    set(h(4),'CData',CData_FR);

%CData = get(h(1),'CData')
%set(h(1),'FaceColor',[1 0 1]);  %pink
%set(h(3),'FaceColor',[0 1 1]);  %teal
%set(h(4),'FaceColor',[1 1 0]);  %yellow

%Plot Body
    h(5) = fill3(x, y, z, 1);       %square siding
    %more shapes if needed
    h(6) = fill3(x1, y1, z1, 2);       %square siding


%% 4. create a group object and parent surfaces
combinedobject = hgtransform('parent',main_vehicle_plot);
c1 = hgtransform('parent',main_vehicle_plot);
c2 = hgtransform('parent',main_vehicle_plot);
c3 = hgtransform('parent',main_vehicle_plot);
c4 = hgtransform('parent',main_vehicle_plot);
c5 = hgtransform('parent',main_vehicle_plot);
c6 = hgtransform('parent',main_vehicle_plot);
c7 = hgtransform('parent',main_vehicle_plot);
c8 = hgtransform('parent',main_vehicle_plot);
c9 = hgtransform('parent',main_vehicle_plot);
c10 = hgtransform('parent',main_vehicle_plot);
set(h(1:6), 'parent', combinedobject)
h1 = copyobj(h,c1);
h2 = copyobj(h,c2);
h3 = copyobj(h,c3);
h4 = copyobj(h,c4);
h5 = copyobj(h,c5);
h6 = copyobj(h,c6);
h7 = copyobj(h,c7);
h8 = copyobj(h,c8);
h9 = copyobj(h,c9);
h10 = copyobj(h,c10);

    %##FrontWheels = hgtransform('parent',myaxes);
    %##set(h(3:4), 'parent', FrontWheels)
drawnow


%% 5. define the motion coordinates
step = 1;               % initialize data stepper
time = 5.001;               % time of simulation display [seconds]
fps = 40;               % Frames/second desired
samples_p_sec = 1000;  % Samples per second of model simulation
               

sample_reduction = int64(samples_p_sec/fps);

total_samples = int64(time*samples_p_sec);
samples_taken = int64(total_samples/sample_reduction);

%car measurements
%Lw - Distance between wheels (LR distance) [m]
%Lf - Distance from front axle to CoG [m]
RHO = sqrt((Lw/2)^2+(Lf)^2);

longitude_s = 1.5;
latitude_s  = .75;

for inc = 1:(samples_taken - 1);
    
    %vehicle movement    
    longitude(inc,1) = VMC(step,1);         %x-direction translation
    latitude(inc,1)  = VMC(step,2);         %y-direction translation
    bearing(inc,1)   = VMC(step,3);         %rotation (degrees)
    S_A(inc,1)       = VMC(step,4);         %Steering Angle
    T_P(inc,1)       = VMC(step,5);         %Throttle Position
    Torque_FL(inc,1) = VMC(step,6);         %Torque FL
    Torque_FR(inc,1) = VMC(step,7);         %Torque FR
    Torque_RL(inc,1) = VMC(step,8);         %Torque RL
    Torque_RR(inc,1) = VMC(step,9);         %Torque RR
    Time(inc,1)      = VMC(step,10);        %Time
    
    altitude(inc,1) = 0;                    %z-direction translation

    %Wheel Steering movement
%     bearing_fw(inc,1) = VMC1(step,1);           %rotation (degrees) SWs WRT the Vehicle
%     
%     [Xs(inc),Ys(inc),Zs(inc)] = pol2cart(bearing(inc)+atan(2),RHO,0)
%     longitude_s(inc,1) = Ys(inc)+longitude(inc,1);          % x-direction translation
%     latitude_s(inc,1) = Xs(inc)+latitude(inc,1);            % y-direction translation
%     
%     
%     FL_spin(inc,1) = VMC2(step,1);
%     FR_spin(inc,1) = VMC2(step,2);
%     RL_spin(inc,1) = VMC2(step,3);
%     RR_spin(inc,1) = VMC2(step,4);
    
    %step data import by samples needed rate
    step = step + sample_reduction;
end
%% 6. animate by using the function makehgtform

% h positions:
 % h(1) = RR
 % h(2) = RL
 % h(3) = FL
 % h(4) = FR
 
for i = 1:(samples_taken - 1)
    
    translation = makehgtform('translate',[latitude(i) longitude(i) altitude(i)]);
    %line drawn from COG of vehicle
    line([latitude(i) latitude(i+1)],[longitude(i) longitude(i+1)],'color',[0 .1 .5]);
    
    %Line drawn from center of FR wheel
    %line([latitude_s(i) latitude_s(i+1)],[longitude_s(i) longitude_s(i+1)],'color',[1 0 1]);
    
    %subplot(2,2,2)          %change graph
    %axis([0,6,-150,150])
    %Draw lines of Torques
    %plot(Time(i),Torque_FL(i),'color',[1 0 1]);
    %hold on
    
%     subplot(2,2,3)
%     axis([0,time,-150,150])
%     line([Time(i) Time(i+1)],[Torque_FL(i) Torque_FL(i+1)],'color',[1 0 1]); %purple
%     line([Time(i) Time(i+1)],[Torque_FR(i) Torque_FR(i+1)],'color',[0 0 1]); %blue
%     
%     subplot(2,2,4)
%     axis([0,time,-150,150])
%     line([Time(i) Time(i+1)],[Torque_RL(i) Torque_RL(i+1)],'color',[1 0 0]); %red
%     line([Time(i) Time(i+1)],[Torque_RR(i) Torque_RR(i+1)],'color',[0 1 1]); %teal
%     
    subplot(2,2,2)
    axis([0,time,-15,15])
    line([Time(i) Time(i+1)],[180/pi*S_A(i) 180/pi*S_A(i+1)],'color',[1 0 0]); %red
    %line([Time(i) Time(i+1)],[Torque_RR(i) Torque_RR(i+1)],'color',[0 1 1]); %teal   
    
    %set(combinedobject, 'matrix', translation);
    subplot(2,2,1)          %change graph
   
    %rotation1 = makehgtform('xrotate', (pi/180)*(bearing(i)));
    %rotation2 = makehgtform('yrotate', (pi/180)*(bearing(i)));
    rotation3 = makehgtform('zrotate', (bearing(i))); 
    set(combinedobject, 'matrix', translation*rotation3);
    
    %use object copy to plot multiple cars on plot?
    %https://www.mathworks.com/help/matlab/ref/hgtransform.html
    car_cnt = 10;
    switch i
        case 1
            set(c1, 'matrix', translation*rotation3);
        case 2*samples_taken/car_cnt
            set(c2, 'matrix', translation*rotation3);
        case 3*samples_taken/car_cnt
            set(c3, 'matrix', translation*rotation3);
        case 4*samples_taken/car_cnt
            set(c4, 'matrix', translation*rotation3);
        case 5*samples_taken/car_cnt
            set(c5, 'matrix', translation*rotation3);
        case 6*samples_taken/car_cnt
            set(c6, 'matrix', translation*rotation3);
        case 7*samples_taken/car_cnt
            set(c7, 'matrix', translation*rotation3);
        case 8*samples_taken/car_cnt
            set(c8, 'matrix', translation*rotation3);
        case 9*samples_taken/car_cnt
            set(c9, 'matrix', translation*rotation3);
        case 10*samples_taken/car_cnt
            set(c10, 'matrix', translation*rotation3);
        
        otherwise
        
    end
    
%     %Rotate Steering wheels on Z-axis
%         zdir = [0 0 1];
%         rotate(h(4),zdir,-bearing_fw(i)*pi/180,[latitude_s(1) longitude_s(1) 0]);
%         rotate(h(3),zdir,-bearing_fw(i)*pi/180,[-latitude_s(1) longitude_s(1) 0]);
        
      %Rotate Wheels due to motion (wheel spin)
        xdir = [1 0 0];
        %rotate(h(1),xdir,-RR_spin(i),[latitude_s(1) -longitude_s(1) 0]);
        %rotate(h(2),xdir,-RL_spin(i),[-latitude_s(1) -longitude_s(1) 0]);
        %rotate(h(3),xdir,-FL_spin(i),[-latitude_s(1) longitude_s(1) 0]);
        %rotate(h(4),xdir,-FR_spin(i),[latitude_s(1) longitude_s(1) 0]);
        
     
    %Statement to keep camera focused on vehicle whilst moving
    %The axes limits are track the vehicle 
    %set(main_vehicle_plot,'ylim',[-10+longitude(i) 10+longitude(i)]);
    %set(main_vehicle_plot,'xlim',[-3+latitude(i) 3+latitude(i)]);
    
    %fixed axes limits

    set(main_vehicle_plot,'ylim',[-5 140]);
    set(main_vehicle_plot,'xlim',[-5 400]);
    
    pause(1/(20000*fps));
    
end
