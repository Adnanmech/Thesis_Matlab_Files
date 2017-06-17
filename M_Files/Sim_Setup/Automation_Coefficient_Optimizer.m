%automate simulations and solve for coefficients yielding best results
%stop simulation command:
%set_param(gcs, 'SimulationCommand', 'stop')

%prevent warnings from showing in command window
warning('off','all');

Simulation_Count = 0; %Counts the simulation iteration number

nsp = 10;  %number of saved control gain points
stop_time = 4.001;   %simulation run time (HAS TO BE CHANGED HERE AND ALSO IN MODEL FILE)
Vx_arr = repmat(100, 1, nsp);
Yaw_Ctrl_Gain_Lowest = repmat(100, 1, nsp);
Slip_Err_P_Gain_Lowest = repmat(100, 1, nsp);
Slip_Err_D_Gain_Lowest = repmat(100, 1, nsp);
Lat_Accel_Err_Gain_Lowest = repmat(100, 1, nsp);

VMC_Vx = repmat(100, int32(stop_time*1000), nsp);
VMC_Vy = repmat(100, int32(stop_time*1000), nsp);
VMC_r  = repmat(100, int32(stop_time*1000), nsp);

sim_pts = 10;
sim_pts_la = 5;
for cntr3=1:sim_pts_la
    for cntr2=1:sim_pts
        for cntr=1:sim_pts
            for cntr1=1:sim_pts
                %update workspace
                whos
                
                %Set up sim values
                AWD_Test_W_FUZ_Control
                
                %make edits to sim values
                Lat_Accel_Err_Gain = (cntr3/(2*sim_pts_la) + 0.5)*1
                Yaw_Ctrl_Gain = (cntr2/sim_pts)*1
                Slip_Err_P_Gain = (cntr1/sim_pts)*1
                Slip_Err_D_Gain = (cntr/sim_pts)*1
                
                %simulate and collect data
                Simulation_Count = Simulation_Count + 1
                %assignin('base', 'Simulation_Count', Simulation_Count)
                sim('All_Combined\AWD_EV_MODEL_rev2.mdl')
                
                %analyze data and make decision
                min_new = min(abs(VMC(:,16)));          %Check min Vx
                if  VMC(3000,16) < 12 ...                %make sure Vx is lower than 12m/s by 3s
                        && max(abs(VMC(:,17))) < 1 ...  %make sure Vy lowoer than 1m/s the entire time.
                        && max(VMC(:,18)) < 0.08;       %make sure yaw rate does not exceed 0.08rad(4.5deg)/s
                    for m = 1:nsp
                        if min_new < Vx_arr(1:m);
                            for s = m:nsp
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
                            Vx_arr_X(1,m) = min_new;
                            Yaw_Ctrl_Gain_Lowest_X(1,m) = Yaw_Ctrl_Gain;
                            Slip_Err_P_Gain_Lowest_X(1,m) = Slip_Err_P_Gain;
                            Slip_Err_D_Gain_Lowest_X(1,m) = Slip_Err_D_Gain;
                            Lat_Accel_Err_Gain_Lowest_X(1,m) = Lat_Accel_Err_Gain;
                            
                            VMC_Vx_X(:,m) = VMC(:,16);
                            VMC_Vy_X(:,m) = VMC(:,17);
                            VMC_r_X (:,m) = VMC(:,18);
                            
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

Filename_ABS = sprintf('Sliding_Mode_ABS_Test_%s.mat', datestr(now,'mm-dd-yyyy_HH-MM'));
save(Filename_ABS);

figure % new figure
hold on
ax1 = subplot(3,1,1); % top subplot
hold on
ax2 = subplot(3,1,2); % bottom subplot
hold on
ax3 = subplot(3,1,3); % bottom subplot

for m = 1:5
    hold on
    plot(ax1,VMC(:,10),VMC_Vx (:,m))
    plot(ax2,VMC(:,10),VMC_Vy (:,m))
    plot(ax3,VMC(:,10),VMC_r (:,m))
end

