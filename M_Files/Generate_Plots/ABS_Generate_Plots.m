%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot Driver inputs in group with separate plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f1 = figure

ThotP = subplot(2,1,1)
plot(VMC(:,10), VMC(:,5))  %Throttle

SAP = subplot(2,1,2)
plot(VMC(:,10), VMC(:,4)*180/3.14)  %Steering Angle

grid(ThotP,'on')
ThotP.XMinorGrid = 'on'
ThotP.YMinorGrid = 'on'
grid(SAP,'on')
SAP.XMinorGrid = 'on'
SAP.YMinorGrid = 'on'

ylim(ThotP,[-1.1 1.1])
ylim(SAP,[-1.1 1.1])

xlabel(SAP,'Time [s]','FontSize',12)

ylabel(ThotP,'Throttle [%]','FontSize',16)
ylabel(SAP,'Steering Angle [°]','FontSize',16)

set(ThotP,'FontSize',12)
set(SAP,'FontSize',12)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot all four slip ratios in group with separate plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f2 = figure

SRFL = subplot(4,1,1)
plot(VMC(:,10), VMC(:,11))  %FL_SR

SRFR = subplot(4,1,2)
plot(VMC(:,10), VMC(:,12))  %FR_SR

SRRL = subplot(4,1,3)
plot(VMC(:,10), VMC(:,13))  %RL_SR

SRRR = subplot(4,1,4)
plot(VMC(:,10), VMC(:,14))  %RR_SR

axis([SRFL SRFR SRRL SRRR],[0 5 -0.21 0.21])
grid(SRFL,'on')
SRFL.XMinorGrid = 'on'
SRFL.YMinorGrid = 'on'
grid(SRFR,'on')
SRFR.XMinorGrid = 'on'
SRFR.YMinorGrid = 'on'
grid(SRRL,'on')
SRRL.XMinorGrid = 'on'
SRRL.YMinorGrid = 'on'
grid(SRRR,'on')
SRRR.XMinorGrid = 'on'
SRRR.YMinorGrid = 'on'

xlabel(SRRR,'Time [s]','FontSize',16)

ylabel(SRFL,'\lambda_{FL}','FontSize',30)
ylabel(SRFR,'\lambda_{FR}','FontSize',30)
ylabel(SRRL,'\lambda_{RL}','FontSize',30)
ylabel(SRRR,'\lambda_{RR}','FontSize',30)

set(SRFL,'FontSize',14)
set(SRFR,'FontSize',14)
set(SRRL,'FontSize',14)
set(SRRR,'FontSize',14)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot all four torque controls in group with separate plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f3 = figure

TauFL = subplot(4,1,1)
plot(VMC(:,10), VMC(:,6))  %FL_Tau

TauFR = subplot(4,1,2)
plot(VMC(:,10), VMC(:,7))  %FR_Tau

TauRL = subplot(4,1,3)
plot(VMC(:,10), VMC(:,8))  %RL_Tau

TauRR = subplot(4,1,4)
plot(VMC(:,10), VMC(:,9))  %RR_Tau

%axis([TauFL TauFR TauRL TauRR],[0 4.902 -0.3 0.11])
grid(TauFL,'on')
TauFL.XMinorGrid = 'on'
TauFL.YMinorGrid = 'on'
grid(TauFR,'on')
TauFR.XMinorGrid = 'on'
TauFR.YMinorGrid = 'on'
grid(TauRL,'on')
TauRL.XMinorGrid = 'on'
TauRL.YMinorGrid = 'on'
grid(TauRR,'on')
TauRR.XMinorGrid = 'on'
TauRR.YMinorGrid = 'on'

xlabel(TauRR,'Time [s]','FontSize',16)

ylabel(TauFL,'\tau_{FL} [Nm]','FontSize',30)
ylabel(TauFR,'\tau_{FR} [Nm]','FontSize',30)
ylabel(TauRL,'\tau_{RL} [Nm]','FontSize',30)
ylabel(TauRR,'\tau_{RR} [Nm]','FontSize',30)

set(TauFL,'FontSize',14)
set(TauFR,'FontSize',14)
set(TauRL,'FontSize',14)
set(TauRR,'FontSize',14)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot basic telemetry in group with separate plots (Vx, Vy, r)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f4 = figure
Vxp = subplot(3,1,1)
plot(VMC(:,10), VMC(:,16))  %Vx

Vyp = subplot(3,1,2)
plot(VMC(:,10), VMC(:,17))  %Vy

rp = subplot(3,1,3)
plot(VMC(:,10), VMC(:,18)*180/pi)  %r
%plot control errors

grid(Vxp,'on')
Vxp.XMinorGrid = 'on'
Vxp.YMinorGrid = 'on'
grid(Vyp,'on')
Vyp.XMinorGrid = 'on'
Vyp.YMinorGrid = 'on'
grid(rp,'on')
rp.XMinorGrid = 'on'
rp.YMinorGrid = 'on'

xlabel(rp,'Time [s]','FontSize',16)

ylabel(Vxp,'V_X [m/s]','FontSize',30)
ylabel(Vyp,'V_Y [m/s]','FontSize',30)
ylabel(rp,'r [°/s]','FontSize',30)

set(Vxp,'FontSize',14)
set(Vyp,'FontSize',14)
set(rp,'FontSize',14)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot errors in group with separate plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f5 = figure

YEP = subplot(2,1,1)
plot(VMC(:,10), VMC(:,15)*180/pi)  %Yaw Error

LAEP = subplot(2,1,2)
plot(VMC(:,10), VMC(:,19))  %Lateral Acceleration Error

grid(YEP,'on')
YEP.XMinorGrid = 'on'
YEP.YMinorGrid = 'on'
grid(LAEP,'on')
LAEP.XMinorGrid = 'on'
LAEP.YMinorGrid = 'on'

%xlim(YEP,[0 4.902])
%xlim(LAEP,[0 4.902])

xlabel(LAEP,'Time [s]','FontSize',16)

ylabel(YEP,'Yaw Error [°/s]','FontSize',30)
ylabel(LAEP,'Lateral Acceleration Error [m/s^2]','FontSize',30)

set(YEP,'FontSize',14)
set(LAEP,'FontSize',14)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Save Figures
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f1.PaperUnits = 'inches';
f1.PaperPosition = [0 0 6 3];
saveas(f1,'D:\SkyDrive\Thesis Paper and Other Works\Thesis LaTeX Rev1\Figures\Rev1\SMC_ABS_T_SA.jpg')

f2.PaperUnits = 'inches';
f2.PaperPosition = [0 0 6 3];
saveas(f2,'D:\SkyDrive\Thesis Paper and Other Works\Thesis LaTeX Rev1\Figures\Rev1\SMC_ABS_SR.jpg')

f3.PaperUnits = 'inches';
f3.PaperPosition = [0 0 6 3];
saveas(f3,'D:\SkyDrive\Thesis Paper and Other Works\Thesis LaTeX Rev1\Figures\Rev1\SMC_ABS_TQ.jpg')

f4.PaperUnits = 'inches';
f4.PaperPosition = [0 0 6 3];
saveas(f4,'D:\SkyDrive\Thesis Paper and Other Works\Thesis LaTeX Rev1\Figures\Rev1\SMC_ABS_VxVyr.jpg')

f5.PaperUnits = 'inches';
f5.PaperPosition = [0 0 6 3];
saveas(f5,'D:\SkyDrive\Thesis Paper and Other Works\Thesis LaTeX Rev1\Figures\Rev1\SMC_ABS_ERRs.jpg')