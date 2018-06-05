%automate simulations and solve for coefficients yielding best results
%stop simulation command:
%set_param(gcs, 'SimulationCommand', 'stop')

%prevent warnings from showing in command window
warning('off','all');

Simulation_Count = 0; %Counts the simulation iteration number
tstart = 200; %time that braking starts in ms
gdp = 0;
nsp = 50;  %number of saved control gain points
stop_time = 5.001;   %simulation run time (HAS TO BE CHANGED HERE AND ALSO IN MODEL FILE)
Vx_arr = repmat(10000, 1, nsp);
Yaw_Ctrl_Gain_Lowest = repmat(100, 1, nsp);
Slip_Err_P_Gain_Lowest = repmat(100, 1, nsp);
%Slip_Err_D_Gain_Lowest = repmat(100, 1, nsp);
%Lat_Accel_Err_Gain_Lowest = repmat(100, 1, nsp);

VMC_Vx = repmat(100, int32(stop_time*1000), nsp);
VMC_Vy = repmat(100, int32(stop_time*1000), nsp);
VMC_r  = repmat(100, int32(stop_time*1000), nsp);

%Set up sim values
addpath('M_Files/Sim_Setup');
run('AWD_Test_W_FUZ_Control.m');

%add paths for sim files
addpath('All_Combined');
addpath('Fuzzy_Controller_Files');

sim_pts = 29;   %20
sim_pts_f_SR = 27;  %10
%for cntr3=sim_pts:-1:1
for cntr=1:sim_pts
    %for cntr1=1:sim_pts_f_SR
        %for cntr1=sim_pts:-1:1
        %update workspace
        %whos
        
        %make edits to sim values
        %Lat_Accel_Err_Gain = 0.2;%(cntr/sim_pts)*1
        Yaw_Ctrl_Gain = 0.2;%(cntr1/sim_pts)*1
        
        %Slip_Err_P_Gain = 35 + 15*cntr
        %Tau_SRC = 1/(2*pi*(31 - cntr1))
        
        Slip_Err_P_Gain = exp(-15 + cntr)
        Tau_SRC = 1/(2*pi*250)
        
        %simulate and collect data
        Simulation_Count = Simulation_Count + 1
        %assignin('base', 'Simulation_Count', Simulation_Count)
        sim('All_Combined\AWD_EV_MODEL_rev2.mdl')%, 'CaptureErrors', 'on')
        
        %Save f_SR values
%        f_SR_Saved(cntr1) = 1/(2*pi*Tau_SRC)
        
        %generate rms error information
        Slip_Err_Gain_Saved(cntr) = Slip_Err_P_Gain
        FL_rms_err(cntr,cntr1) = rms(VMC(tstart:find(VMC(:,16)<0.01,1),11) - -0.1)
        FR_rms_err(cntr,cntr1) = rms(VMC(tstart:find(VMC(:,16)<0.01,1),12) - -0.1)
        RL_rms_err(cntr,cntr1) = rms(VMC(tstart:find(VMC(:,16)<0.01,1),13) - -0.1)
        RR_rms_err(cntr,cntr1) = rms(VMC(tstart:find(VMC(:,16)<0.01,1),14) - -0.1)
        
        %analyze data and make decision
        min_new = find(VMC(:,16)<0.01,1);          %Check min Vx
        if  VMC(3000,16) < 16 ...               %make sure Vx is lower than 12m/s by 3s
                && max(abs(VMC(:,17))) < 2 ...  %make sure Vy lowoer than 1m/s the entire time.
                && max(VMC(:,18)) < 0.18;       %make sure yaw rate does not exceed 0.08rad(4.5deg)/s
            for num = 1:(nsp-1)
                if min_new < Vx_arr(num);
                    for s = num:nsp
                        Vx_arr_X(1,s+1) = Vx_arr(1,s);
                        Yaw_Ctrl_Gain_Lowest_X(1,s+1) = Yaw_Ctrl_Gain_Lowest(1,s);
                        Slip_Err_P_Gain_Lowest_X(1,s+1) = Slip_Err_P_Gain_Lowest(1,s);
                        %Slip_Err_D_Gain_Lowest_X(1,s+1) = Slip_Err_D_Gain_Lowest(1,s);
                        %Lat_Accel_Err_Gain_Lowest_X(1,s+1) = Lat_Accel_Err_Gain_Lowest(1,s);
                        
                        VMC_Vx_X(:,s+1) = VMC_Vx(:,s);
                        VMC_Vy_X(:,s+1) = VMC_Vy(:,s);
                        VMC_r_X (:,s+1) = VMC_r (:,s);
                    end
                    %Place value in dummy (X) constructed arrays
                    Vx_arr_X(1,num) = min_new;
                    Yaw_Ctrl_Gain_Lowest_X(1,num) = Yaw_Ctrl_Gain;
                    Slip_Err_P_Gain_Lowest_X(1,num) = Slip_Err_P_Gain;
                    %Slip_Err_D_Gain_Lowest_X(1,num) = Slip_Err_D_Gain;
                    %Lat_Accel_Err_Gain_Lowest_X(1,num) = Lat_Accel_Err_Gain;
                    
                    VMC_Vx_X(:,num) = VMC(:,16);
                    VMC_Vy_X(:,num) = VMC(:,17);
                    VMC_r_X (:,num) = VMC(:,18);
                    
                    %set newly constructed arrays to variable
                    Vx_arr = Vx_arr_X;
                    Yaw_Ctrl_Gain_Lowest = Yaw_Ctrl_Gain_Lowest_X;
                    Slip_Err_P_Gain_Lowest = Slip_Err_P_Gain_Lowest_X;
                    %Slip_Err_D_Gain_Lowest = Slip_Err_D_Gain_Lowest_X;
                    %Lat_Accel_Err_Gain_Lowest = Lat_Accel_Err_Gain_Lowest_X;
                    
                    Gains(1,:) = Yaw_Ctrl_Gain_Lowest_X;
                    Gains(2,:) = Slip_Err_P_Gain_Lowest_X;
                    %Gains(3,:) = Slip_Err_D_Gain_Lowest_X;
                    %Gains(4,:) = Lat_Accel_Err_Gain_Lowest_X;
                    
                    VMC_Vx = VMC_Vx_X;
                    VMC_Vy = VMC_Vy_X;
                    VMC_r  = VMC_r_X;
                    
                    gdp = gdp + 1;
                    break
                else
                    dummy=1;
                end
            end
            
        else
            dummy=1;
            
        end
        
        %end
    %end
end
%end

%plot Pgain v f_SR v rms error for all wheels
figure % new figure
Gaintt = Slip_Err_Gain_Saved';
ftt = f_SR_Saved';
figure % new figure
hold on
ax011 = subplot(4,1,1); % top subplot
surfc(ftt, Gaintt, FL_rms_err)
hold on

%find minimum points
nminpts = 20;   %set number of min pts to collect
FL_rms_err_1col = FL_rms_err(:)
[tempmins,originalpos] = sort( FL_rms_err(:), 'ascend' );
mins = tempmins(1:nminpts)
pos=originalpos(1:nminpts)

for acnt = 1:nminpts
    indicCol(acnt,1) = idivide(int32(pos(acnt,1)), int32(sim_pts),'ceil')
    rem1 = mod(int32(pos(acnt,1)),int32(sim_pts))
    if(rem1 == 0); 
        indicRow(acnt,1) = sim_pts
    else
        indicRow(acnt,1) = rem1
    end
    hp1 = scatter3(ftt(indicCol(acnt,1)),Gaintt(indicRow(acnt,1)),FL_rms_err(indicRow(acnt,1),indicCol(acnt,1)),'filled');
    hp1.SizeData = 150;
end
xlabel('freq');
ylabel('Gain');

ax012 = subplot(4,1,2); % bottom subplot
surfc(ftt, Gaintt, FR_rms_err)
hold on

%find minimum points
FR_rms_err_1col = FR_rms_err(:)
[tempmins,originalpos] = sort( FR_rms_err(:), 'ascend' );
mins = tempmins(1:nminpts)
pos=originalpos(1:nminpts)

for acnt = 1:nminpts
    indicCol(acnt,1) = idivide(int32(pos(acnt,1)), int32(sim_pts),'ceil')
    rem1 = mod(int32(pos(acnt,1)),int32(sim_pts))
    if(rem1 == 0); 
        indicRow(acnt,1) = sim_pts
    else
        indicRow(acnt,1) = rem1
    end
    hp1 = scatter3(ftt(indicCol(acnt,1)),Gaintt(indicRow(acnt,1)),FR_rms_err(indicRow(acnt,1),indicCol(acnt,1)),'filled');
    hp1.SizeData = 150;
end
xlabel('freq');
ylabel('Gain');

ax013 = subplot(4,1,3); % bottom subplot
surfc(ftt, Gaintt, RL_rms_err)
hold on

%find minimum points
RL_rms_err_1col = RL_rms_err(:)
[tempmins,originalpos] = sort( RL_rms_err(:), 'ascend' );
mins = tempmins(1:nminpts)
pos=originalpos(1:nminpts)

for acnt = 1:nminpts
    indicCol(acnt,1) = idivide(int32(pos(acnt,1)), int32(sim_pts),'ceil')
    rem1 = mod(int32(pos(acnt,1)),int32(sim_pts))
    if(rem1 == 0); 
        indicRow(acnt,1) = sim_pts
    else
        indicRow(acnt,1) = rem1
    end
    hp1 = scatter3(ftt(indicCol(acnt,1)),Gaintt(indicRow(acnt,1)),RL_rms_err(indicRow(acnt,1),indicCol(acnt,1)),'filled');
    hp1.SizeData = 150;
end
xlabel('freq');
ylabel('Gain');

ax014 = subplot(4,1,4); % bottom subplot
hold on
surfc(ftt, Gaintt, RR_rms_err)

%find minimum points
RR_rms_err_1col = RR_rms_err(:)
[tempmins,originalpos] = sort( RR_rms_err(:), 'ascend' );
mins = tempmins(1:nminpts)
pos=originalpos(1:nminpts)

for acnt = 1:nminpts
    indicCol(acnt,1) = idivide(int32(pos(acnt,1)), int32(sim_pts),'ceil')
    rem1 = mod(int32(pos(acnt,1)),int32(sim_pts))
    if(rem1 == 0); 
        indicRow(acnt,1) = sim_pts
    else
        indicRow(acnt,1) = rem1
    end
    hp1 = scatter3(ftt(indicCol(acnt,1)),Gaintt(indicRow(acnt,1)),RR_rms_err(indicRow(acnt,1),indicCol(acnt,1)),'filled');
    hp1.SizeData = 150;
end
xlabel('freq');
ylabel('Gain');

%save rms error surface plot
Filename_3d_SURFC_fig_rms_err = sprintf('Sliding_Mode_Control_ABS_Test_rms_err_3d_SURFC_fig_175Nm_%s.fig', datestr(now,'mm-dd-yyyy_HH-MM'));
savefig(Filename_3d_SURFC_fig_rms_err);

%create total error plot
figure
Tot_rms_err = FL_rms_err + FR_rms_err + RL_rms_err + RR_rms_err;
hold on
surfc(ftt, Gaintt, Tot_rms_err)
hold on

%find minimum points
Tot_rms_err_1col = Tot_rms_err(:)
[tempmins,originalpos] = sort( Tot_rms_err(:), 'ascend' );
mins = tempmins(1:nminpts)
pos=originalpos(1:nminpts)

for acnt = 1:nminpts
    indicCol(acnt,1) = idivide(int32(pos(acnt,1)), int32(sim_pts),'ceil')
    rem1 = mod(int32(pos(acnt,1)),int32(sim_pts))
    if(rem1 == 0); 
        indicRow(acnt,1) = sim_pts
    else
        indicRow(acnt,1) = rem1
    end
    hp1 = scatter3(ftt(indicCol(acnt,1)),Gaintt(indicRow(acnt,1)),Tot_rms_err(indicRow(acnt,1),indicCol(acnt,1)),'filled');
    hp1.SizeData = 150;
    hold on
end
xlabel('freq');
ylabel('Gain');

%save total rms error surface plot
Filename_3d_SURFC_fig_Tot_rms_err = sprintf('Sliding_Mode_Control_ABS_Test_rms_err_3d_SURFC_fig_175Nm_%s.fig', datestr(now,'mm-dd-yyyy_HH-MM'));
savefig(Filename_3d_SURFC_fig_Tot_rms_err);

%plot rms errors vs P Gain
figure % new figure
hold on
ax11 = subplot(4,1,1); % top subplot
hold on
ax12 = subplot(4,1,2); % bottom subplot
hold on
ax13 = subplot(4,1,3); % bottom subplot
hold on
ax14 = subplot(4,1,4); % bottom subplot
hold on
plot(ax11,Slip_Err_Gain_Saved,FL_rms_err)
plot(ax12,Slip_Err_Gain_Saved,FR_rms_err)
plot(ax13,Slip_Err_Gain_Saved,RL_rms_err)
plot(ax14,Slip_Err_Gain_Saved,RR_rms_err)

%save rms error plot
Filename_fig_rms_err = sprintf('Sliding_Mode_Control_ABS_Test_rms_err_fig_175Nm_%s.fig', datestr(now,'mm-dd-yyyy_HH-MM'));
savefig(Filename_fig_rms_err);

%save workspace to file

Filename_ABS = sprintf('Sliding_Mode_Control_ABS_Test_ABS_Test_175Nm_%s.mat', datestr(now,'mm-dd-yyyy_HH-MM'));
save(Filename_ABS);

figure % new figure
hold on
ax1 = subplot(3,1,1); % top subplot
hold on
ax2 = subplot(3,1,2); % bottom subplot
hold on
ax3 = subplot(3,1,3); % bottom subplot

if gdp > nsp
    gdp = nsp;
end
for m = 1:gdp
    hold on
    plot(ax1,VMC(:,10),VMC_Vx (:,m))
    plot(ax2,VMC(:,10),VMC_Vy (:,m))
    plot(ax3,VMC(:,10),VMC_r (:,m))
end

Filename_fig = sprintf('Sliding_Mode_Control_ABS_Test_fig_175Nm_%s.fig', datestr(now,'mm-dd-yyyy_HH-MM'));
savefig(Filename_fig);