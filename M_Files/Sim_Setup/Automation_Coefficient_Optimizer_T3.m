%automate simulations and solve for coefficients yielding best results
%Sliding Mode VLC Test
%stop simulation command:
%set_param(gcs, 'SimulationCommand', 'stop')

%prevent warnings from showing in command window
warning('off','all');

Simulation_Count = 0; %Counts the simulation iteration number

gdp = 0;   %Number of actual saved control gain points (initialized to zero)
nsp = 50;  %Max number of saved control gain points
stop_time = 5.101;   %simulation run time (HAS TO BE CHANGED HERE AND ALSO IN MODEL FILE)
run_time = stop_time - 0.001;
Vx_arr = repmat(100, 1, nsp);
Vx_arr_X = repmat(100, 1, nsp);
min_rms_YE = repmat(100, 1, nsp);
min_rms_YE_X = repmat(100, 1, nsp);
min_rms_LAE = repmat(100, 1, nsp);
min_rms_LAE_X = repmat(100, 1, nsp);
Lat_Accel_Err_Gain_Lowest = repmat(100, 1, nsp);
Lat_Accel_Err_Gain_Lowest_X = repmat(100, 1, nsp);
Slip_Err_P_Gain_Lowest = repmat(100, 1, nsp);
Slip_Err_P_Gain_Lowest_X = repmat(100, 1, nsp);
Yaw_Err_P_Gain_Lowest = repmat(100, 1, nsp);
Yaw_Err_P_Gain_Lowest_X = repmat(100, 1, nsp);
fc_SR_Lowest = repmat(100, 1, nsp);
fc_SR_X = repmat(100, 1, nsp);

VMC_Vx  = repmat(100, int32(stop_time*1000), nsp);
VMC_Vy  = repmat(100, int32(stop_time*1000), nsp);
VMC_r   = repmat(100, int32(stop_time*1000), nsp);
VMC_YE  = repmat(100, int32(stop_time*1000), nsp);
VMC_LAE = repmat(100, int32(stop_time*1000), nsp);

%Set up sim values
addpath('M_Files/Sim_Setup');
run('AWD_Test_W_FUZ_Control.m');

%Add path for sim files
addpath('All_Combined');
addpath('Fuzzy_Controller_Files');


%sim_pts_yaw_P = 45;
%sim_pts_la = 41;

sim_pts_SRE_P = 51;
sim_pts_fc_SR = 19;

%for cntr3=sim_pts:-1:1
    %for cntr2=sim_pts:-1:1
       for cntr1=1:sim_pts_fc_SR
            for cntr=1:sim_pts_SRE_P
                %update workspace
                %whos
                
                %make edits to sim values
                Lat_Accel_Err_Gain = 1
                Slip_Err_P_Gain = -5000 + 10000*cntr
                Yaw_Err_P_Gain = 10
                Tau_SRC = 1/(2*pi*(2 + cntr1*2))
                fc_SR = 1/(2*pi*Tau_SRC)
                %simulate and collect data
                Simulation_Count = Simulation_Count + 1
                %assignin('base', 'Simulation_Count', Simulation_Count)
                sim('All_Combined\AWD_EV_MODEL_rev2.mdl')%,'CaptureErrors', 'on')
                
                %generate yaw error rms info
                yaw_rms_err(cntr,cntr1) = rms(VMC(:,15));
                la_rms_err(cntr,cntr1) = rms(VMC(:,19));
                Yaw_Err_P_Gain_Saved(cntr) = Yaw_Err_P_Gain;
                Lat_Accel_Err_Gain_Saved(cntr1) = Lat_Accel_Err_Gain;
                Slip_Err_P_Gain_Saved(cntr) = Slip_Err_P_Gain;
                fc_SR_Saved(cntr1) = fc_SR;
                
                
                %analyze data and make decision
				min_new = min(abs(VMC(:,16)));          %Check min Vx
                min_rms_YE_new = rms(VMC(:,15));  %Check min rms Yaw error
                YE_Saved(Simulation_Count) = min_rms_YE_new
                min_rms_LAE_new = rms(VMC(:,19)); %Check min rms Yaw error
                LAE_Saved(Simulation_Count) = min_rms_LAE_new
                
                min_rms_YE
                min_rms_LAE
                
                if  VMC(3000,16) < 24 ...                %make sure Vx is lower than 12m/s by 3s
                        && max(abs(VMC(:,17))) < 2 ...  %make sure Vy lowoer than 1m/s the entire time.
                        && max(VMC(:,18)) < 0.18;       %make sure yaw rate does not exceed 0.08rad(4.5deg)/s
                    for num = 1:(nsp-1)
                        if min_new < Vx_arr(num);
                            for s = num:nsp
							    Vx_arr_X(1,s+1) = Vx_arr(1,s);
                                min_rms_YE_X(1,s+1) = min_rms_YE(1,s);
                                min_rms_LAE_X(1,s+1) = min_rms_LAE(1,s);
                                Lat_Accel_Err_Gain_Lowest_X(1,s+1) = Lat_Accel_Err_Gain_Lowest(1,s);
                                Slip_Err_P_Gain_Lowest_X(1,s+1) = Slip_Err_P_Gain_Lowest(1,s);
                                Yaw_Err_P_Gain_Lowest_X(1,s+1) = Yaw_Err_P_Gain_Lowest(1,s);
                                fc_SR_X(1,s+1) = fc_SR_Lowest(1,s);
                                
                                VMC_Vx_X(:,s+1) = VMC_Vx(:,s);
                                VMC_Vy_X(:,s+1) = VMC_Vy(:,s);
                                VMC_r_X (:,s+1) = VMC_r (:,s);
                                VMC_YE_X(:,s+1) = VMC_YE(:,s);
                                VMC_LAE_X(:,s+1) = VMC_LAE(:,s);
                            end
                            %Place value in 1st slot of dummy (X) arrays
							Vx_arr_X(1,num) = min_new;
                            min_rms_YE_X(1,num) = min_rms_YE_new;
                            min_rms_LAE_X(1,num) = min_rms_LAE_new;
                            Lat_Accel_Err_Gain_Lowest_X(1,num) = Lat_Accel_Err_Gain;
                            Slip_Err_P_Gain_Lowest_X(1,num) = Slip_Err_P_Gain;
                            Yaw_Err_P_Gain_Lowest_X(1,num) = Yaw_Err_P_Gain;
                            fc_SR_X(1,num) = fc_SR;
                            
                            VMC_Vx_X(:,num) = VMC(:,16);
                            VMC_Vy_X(:,num) = VMC(:,17);
                            VMC_r_X (:,num) = VMC(:,18);
                            VMC_YE_X(:,num) = VMC(:,15);
                            VMC_LAE_X(:,num) = VMC(:,19);
                            
                            %set newly constructed arrays to variable
							Vx_arr = Vx_arr_X;
                            min_rms_YE = min_rms_YE_X;
                            min_rms_LAE = min_rms_LAE_X;
                            Lat_Accel_Err_Gain_Lowest = Lat_Accel_Err_Gain_Lowest_X;
                            Slip_Err_P_Gain_Lowest = Slip_Err_P_Gain_Lowest_X;
                            Yaw_Err_P_Gain_Lowest = Yaw_Err_P_Gain_Lowest_X;
                            fc_SR_Lowest = fc_SR_X;
                            
                            Gains(1,:) = Lat_Accel_Err_Gain_Lowest_X;
                            Gains(2,:) = Slip_Err_P_Gain_Lowest_X;
                            Gains(3,:) = Yaw_Err_P_Gain_Lowest_X;
                            Gains(4,:) = fc_SR_X;
                            
                            VMC_Vx = VMC_Vx_X;
                            VMC_Vy = VMC_Vy_X;
                            VMC_r  = VMC_r_X;
                            VMC_YE = VMC_YE_X;
                            VMC_LAE = VMC_LAE_X;
                            
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
    %end
%end

%save workspace to file
Filename_mat = sprintf('Sliding_Mode_Control_CSA_Test_T3_%s.mat', datestr(now,'mm-dd-yyyy_HH-MM'));
save(Filename_mat);

figure
%set(gca, 'YScale', 'log')
%set(gca, 'XScale', 'log')
hold on
PGaintt = Yaw_Err_P_Gain_Saved';
LAGaintt = Lat_Accel_Err_Gain_Saved';
SREGaintt = Slip_Err_P_Gain_Saved';
FCSRtt = fc_SR_Saved';

surfc(FCSRtt, SREGaintt, yaw_rms_err)
%zlim([0.02 0.09])
hold on
xlabel('Fc')
ylabel('SRE Gain')
zlabel('rms(YE)')

%scatter(Yaw_Err_P_Gain_Saved, yaw_rms_err)

%Save figure
Filename_yrmserr_v_ygain_fig = sprintf('Sliding_Mode_Control_CSA_Test_yrmserr_v_SRP_v_SRfc_T3_fig_%s.fig', datestr(now,'mm-dd-yyyy_HH-MM'));
savefig(Filename_yrmserr_v_ygain_fig);

figure
%set(gca, 'YScale', 'log')
%set(gca, 'XScale', 'log')
hold on
PGaintt = Yaw_Err_P_Gain_Saved';
LAGaintt = Lat_Accel_Err_Gain_Saved';
surfc(FCSRtt, SREGaintt, la_rms_err)
%zlim([0.02 0.09])
hold on
xlabel('Fc')
ylabel('SRE Gain')
zlabel('rms(LAE)')

%scatter(Yaw_Err_P_Gain_Saved, yaw_rms_err)

%Save figure
Filename_yrmserr_v_ygain_fig = sprintf('Sliding_Mode_Control_CSA_Test_yrmserr_v_SRP_v_SRfc_T3_fig_%s.fig', datestr(now,'mm-dd-yyyy_HH-MM'));
savefig(Filename_yrmserr_v_ygain_fig);

figure % new figure
hold on
ax1 = subplot(4,1,1); % top subplot
hold on
ax2 = subplot(4,1,2); % bottom subplot
hold on
ax3 = subplot(4,1,3); % bottom subplot
hold on
ax4 = subplot(4,1,4); % bottom subplot
hold on
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

Filename_fig = sprintf('Sliding_Mode_Control_CSA_Test_T3_fig_%s.fig', datestr(now,'mm-dd-yyyy_HH-MM'));
savefig(Filename_fig);