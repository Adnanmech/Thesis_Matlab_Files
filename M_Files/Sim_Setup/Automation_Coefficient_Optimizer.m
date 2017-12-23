%automate simulations and solve for coefficients yielding best results
%stop simulation command:
%set_param(gcs, 'SimulationCommand', 'stop')

%prevent warnings from showing in command window
warning('off','all');

Simulation_Count = 0; %Counts the simulation iteration number
gdp = 0;
nsp = 10;  %number of saved control gain points
stop_time = 4.101;   %simulation run time (HAS TO BE CHANGED HERE AND ALSO IN MODEL FILE)
Vx_arr = repmat(100, 1, nsp);
Yaw_Ctrl_Gain_Lowest = repmat(100, 1, nsp);
Slip_Err_P_Gain_Lowest = repmat(100, 1, nsp);
Slip_Err_D_Gain_Lowest = repmat(100, 1, nsp);
Lat_Accel_Err_Gain_Lowest = repmat(100, 1, nsp);

VMC_Vx = repmat(100, int32(stop_time*1000), nsp);
VMC_Vy = repmat(100, int32(stop_time*1000), nsp);
VMC_r  = repmat(100, int32(stop_time*1000), nsp);

%Set up sim values
addpath('M_Files/Sim_Setup');
run('AWD_Test_W_FUZ_Control.m');

%add paths for sim files
addpath('All_Combined');
addpath('Fuzzy_Controller_Files');

sim_pts = 10;
sim_pts_la = 10;
for cntr3=sim_pts:-1:1
    for cntr2=sim_pts:-1:1
        for cntr=sim_pts:-1:1
            for cntr1=sim_pts:-1:1
                %update workspace
                %whos
                
                %make edits to sim values
                Lat_Accel_Err_Gain = (cntr/sim_pts)*2
                Yaw_Ctrl_Gain = (cntr1/sim_pts)*2
                Slip_Err_P_Gain = (cntr2/sim_pts)*2
                Slip_Err_D_Gain = (cntr3/sim_pts)*2
                
                %simulate and collect data
                Simulation_Count = Simulation_Count + 1
                %assignin('base', 'Simulation_Count', Simulation_Count)
                sim('All_Combined\AWD_EV_MODEL_rev2.mdl')%, 'CaptureErrors', 'on')
                
                %analyze data and make decision
                min_new = min(abs(VMC(:,16)));          %Check min Vx
                if  VMC(3000,16) < 24 ...                %make sure Vx is lower than 12m/s by 3s
                        && max(abs(VMC(:,17))) < 2 ...  %make sure Vy lowoer than 1m/s the entire time.
                        && max(VMC(:,18)) < 0.18;       %make sure yaw rate does not exceed 0.08rad(4.5deg)/s
                    if min_new < Vx_arr(1);
                        for s = 1:(nsp-1)
                            Vx_arr_X(1,s+1) = Vx_arr(1,s);
                            Yaw_Ctrl_Gain_Lowest_X(1,s+1) = Yaw_Ctrl_Gain_Lowest(1,s);
                            Slip_Err_P_Gain_Lowest_X(1,s+1) = Slip_Err_P_Gain_Lowest(1,s);
                            Slip_Err_D_Gain_Lowest_X(1,s+1) = Slip_Err_D_Gain_Lowest(1,s);
                            Lat_Accel_Err_Gain_Lowest_X(1,s+1) = Lat_Accel_Err_Gain_Lowest(1,s);
                            
                            VMC_Vx_X(:,s+1) = VMC_Vx(:,s);
                            VMC_Vy_X(:,s+1) = VMC_Vy(:,s);
                            VMC_r_X (:,s+1) = VMC_r (:,s);
                        end
                        %Place value in dummy (X) constructed arrays
                        Vx_arr_X(1,1) = min_new;
                        Yaw_Ctrl_Gain_Lowest_X(1,1) = Yaw_Ctrl_Gain;
                        Slip_Err_P_Gain_Lowest_X(1,1) = Slip_Err_P_Gain;
                        Slip_Err_D_Gain_Lowest_X(1,1) = Slip_Err_D_Gain;
                        Lat_Accel_Err_Gain_Lowest_X(1,1) = Lat_Accel_Err_Gain;
                        
                        VMC_Vx_X(:,1) = VMC(:,16);
                        VMC_Vy_X(:,1) = VMC(:,17);
                        VMC_r_X (:,1) = VMC(:,18);
                        
                        %set newly constructed arrays to variable
                        Vx_arr = Vx_arr_X;
                        Yaw_Ctrl_Gain_Lowest = Yaw_Ctrl_Gain_Lowest_X;
                        Slip_Err_P_Gain_Lowest = Slip_Err_P_Gain_Lowest_X;
                        Slip_Err_D_Gain_Lowest = Slip_Err_D_Gain_Lowest_X;
                        Lat_Accel_Err_Gain_Lowest = Lat_Accel_Err_Gain_Lowest_X;
                        Gains(1,:) = Yaw_Ctrl_Gain_Lowest_X;
                        Gains(2,:) = Slip_Err_P_Gain_Lowest_X;
                        Gains(3,:) = Slip_Err_D_Gain_Lowest_X;
                        Gains(4,:) = Lat_Accel_Err_Gain_Lowest_X;
                        
                        VMC_Vx = VMC_Vx_X;
                        VMC_Vy = VMC_Vy_X;
                        VMC_r  = VMC_r_X;
                        
                        gdp = gdp + 1;
                        break
                    else
                        dummy=1;
                    end
                    
                    
                else
                    dummy=1;
                    
                end
            end
        end
    end
end

%save workspace to file

Filename_ABS = sprintf('Sliding_Mode_Control_Split-u_Test_%s.mat', datestr(now,'mm-dd-yyyy_HH-MM'));
save(Filename_ABS);

figure % new figure
hold on
ax1 = subplot(3,1,1); % top subplot
hold on
ax2 = subplot(3,1,2); % bottom subplot
hold on
ax3 = subplot(3,1,3); % bottom subplot

if gdp > 10
    gdp = 10;
end 
for m = 1:gdp
    hold on
    plot(ax1,VMC(:,10),VMC_Vx (:,m))
    plot(ax2,VMC(:,10),VMC_Vy (:,m))
    plot(ax3,VMC(:,10),VMC_r (:,m))
end

Filename_fig = sprintf('Sliding_Mode_Control_Split-u_Test_fig_%s.fig', datestr(now,'mm-dd-yyyy_HH-MM'));
savefig(Filename_fig);