%automate simulations and solve for coefficients yielding best results
%Fuzzy Controller Split-u Test
%stop simulation command:
%set_param(gcs, 'SimulationCommand', 'stop')

%prevent warnings from showing in command window
warning('off','all');

Simulation_Count = 0; %Counts the simulation iteration number

gdp = 0;   %Number of actual saved control gain points (initialized to zero)
nsp = 50;  %Max number of saved control gain points
stop_time = 5.001;   %simulation run time (HAS TO BE CHANGED HERE AND ALSO IN MODEL FILE)
min_Vx = repmat(100, 1, nsp);
min_Vx_X = repmat(100, 1, nsp);
min_rms_YE = repmat(100, 1, nsp);
min_rms_YE_X = repmat(100, 1, nsp);
Yaw_Ctrl_Gain_Lowest = repmat(100, 1, nsp);
Yaw_Ctrl_Gain_Lowest_X = repmat(100, 1, nsp);
dYaw_Ctrl_Gain_Lowest = repmat(100, 1, nsp);
dYaw_Ctrl_Gain_Lowest_X = repmat(100, 1, nsp);
Slip_Ratio_Ctrl_Gain_Lowest = repmat(100, 1, nsp);
Slip_Ratio_Ctrl_Gain_Lowest_X = repmat(100, 1, nsp);
Wheel_Accel_Ctrl_Gain_Lowest = repmat(100, 1, nsp);
Wheel_Accel_Ctrl_Gain_Lowest_X = repmat(100, 1, nsp);

VMC_Vx = repmat(100, stop_time*1000, nsp);
VMC_Vy = repmat(100, stop_time*1000, nsp);
VMC_r  = repmat(100, stop_time*1000, nsp);
VMC_YE = repmat(100, stop_time*1000, nsp);

%Set up sim values
addpath('M_Files/Sim_Setup');
run('AWD_Test_W_FUZ_Control.m');

%Add path for sim files
addpath('All_Combined');
addpath('Fuzzy_Controller_Files');

sim_pts_slip = 8;
sim_pts_wa = 8;
sim_pts_yaw = 8;
sim_pts_dyaw = 6;

for cntr=1:sim_pts_slip
    for cntr1=sim_pts_wa:-1:1
        for cntr2=1:sim_pts_yaw
            for cntr3=1:sim_pts_dyaw
                %update workspace
                %whos
                
                %make edits to sim values
                Yaw_Ctrl_Gain = cntr2 - 0.9
                dYaw_Ctrl_Gain = cntr3*0.066 - 0.046
                Slip_Ratio_Ctrl_Gain = cntr*2 - 1
                Wheel_Accel_Ctrl_Gain = cntr1*0.125
                
                %simulate and collect data
                Simulation_Count = Simulation_Count + 1
                %assignin('base', 'Simulation_Count', Simulation_Count)
                sim('All_Combined/AWD_EV_MODEL_rev2.mdl')%,'CaptureErrors', 'on')
                
                %analyze data and make decision
                min_Vx_new = min(abs(VMC(:,16)));  %Check min Vx
                Vx_Saved(Simulation_Count) = min_Vx_new
                min_Vx
                if  VMC(5000,16) < 25 ...            %make sure velocity is lower than 17m/s by 6s
                        && max(abs(VMC(:,17))) < 2 ...     %make sure Vy lower than 1m/s the entire time.
                        && max(VMC(:,18)) < 0.18;    % make sure yaw rate does not exceed 0.08rad(4.5deg)/s
                    for num = 1:(nsp-1)
                        if min_Vx_new < min_Vx(num);
                            for s = num:nsp
                                min_Vx_X(1,s+1) = min_Vx(1,s);
                                Yaw_Ctrl_Gain_Lowest_X(1,s+1) = Yaw_Ctrl_Gain_Lowest(1,s);
                                dYaw_Ctrl_Gain_Lowest_X(1,s+1) = dYaw_Ctrl_Gain_Lowest(1,s);
                                Slip_Ratio_Ctrl_Gain_Lowest_X(1,s+1) = Slip_Ratio_Ctrl_Gain_Lowest(1,s);
                                Wheel_Accel_Ctrl_Gain_Lowest_X(1,s+1) = Wheel_Accel_Ctrl_Gain_Lowest(1,s);
                                
                                VMC_Vx_X(:,s+1) = VMC_Vx(:,s);
                                VMC_Vy_X(:,s+1) = VMC_Vy(:,s);
                                VMC_r_X (:,s+1) = VMC_r (:,s);
                                VMC_YE_X(:,s+1) = VMC_YE(:,s);
                            end
                            %Place value in 1st slot of dummy (X) arrays
                            min_Vx_X(1,num) = min_Vx_new;
                            Yaw_Ctrl_Gain_Lowest_X(1,num) = Yaw_Ctrl_Gain;
                            dYaw_Ctrl_Gain_Lowest_X(1,num) = dYaw_Ctrl_Gain;
                            Slip_Ratio_Ctrl_Gain_Lowest_X(1,num) = Slip_Ratio_Ctrl_Gain;
                            Wheel_Accel_Ctrl_Gain_Lowest_X(1,num) = Wheel_Accel_Ctrl_Gain;
                            
                            VMC_Vx_X(:,num) = VMC(:,16);
                            VMC_Vy_X(:,num) = VMC(:,17);
                            VMC_r_X (:,num) = VMC(:,18);
                            VMC_YE_X(:,num) = VMC(:,15);
                            
                            %set newly constructed arrays to variable
                            min_Vx = min_Vx_X;
                            Yaw_Ctrl_Gain_Lowest = Yaw_Ctrl_Gain_Lowest_X;
                            dYaw_Ctrl_Gain_Lowest = dYaw_Ctrl_Gain_Lowest_X;
                            Slip_Ratio_Ctrl_Gain_Lowest = Slip_Ratio_Ctrl_Gain_Lowest_X;
                            Wheel_Accel_Ctrl_Gain_Lowest = Wheel_Accel_Ctrl_Gain_Lowest_X;
                            
                            Gains(1,:) = Yaw_Ctrl_Gain_Lowest_X;
                            Gains(2,:) = Slip_Ratio_Ctrl_Gain_Lowest_X;
                            Gains(3,:) = Wheel_Accel_Ctrl_Gain_Lowest_X;
                            Gains(4,:) = dYaw_Ctrl_Gain_Lowest_X;
                            
                            VMC_Vx = VMC_Vx_X;
                            VMC_Vy = VMC_Vy_X;
                            VMC_r  = VMC_r_X;
                            VMC_YE = VMC_YE_X;
                            
                            gdp = gdp + 1;
                            
                            break
                        else
                            dummy=1;
                        end
                    end
                    
                else
                    dummy=1;
                end
            end
        end
    end
end

%save workspace to file

Filename_mat = sprintf('Fuzzy_Control_Split-u_Test_T1_%s.mat', datestr(now,'mm-dd-yyyy_HH-MM'));
save(Filename_mat);

figure % new figure
hold on
ax1 = subplot(4,1,1); % top subplot
hold on
ax2 = subplot(4,1,2); % bottom subplot
hold on
ax3 = subplot(4,1,3); % bottom subplot
hold on
ax4 = subplot(4,1,4); % bottom subplot

if gdp > nsp
    gdp = nsp;
end

for m = 1:gdp
    hold on
    plot(ax1,VMC(:,10),VMC_Vx (:,m))
    plot(ax2,VMC(:,10),VMC_Vy (:,m))
    plot(ax3,VMC(:,10),VMC_r (:,m))
    plot(ax4,VMC(:,10),VMC_YE (:,m))
end

Filename_fig = sprintf('Fuzzy_Control_Split-u_T1_fig_%s.fig', datestr(now,'mm-dd-yyyy_HH-MM'));
savefig(Filename_fig);