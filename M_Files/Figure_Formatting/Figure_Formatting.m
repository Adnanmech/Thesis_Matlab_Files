%commands to label pictures

%\, used for spacings

%plotmf(Fuzzy_AYC_Ctl,'input',1)
plotmf(Fuzzy_AYC_Ctl,'input',1)
xlabel('$e_{\dot{r}}$','FontSize',20,'Interpreter','latex');
ylabel('$Degree\, of\, Membership$','FontSize',20,'Interpreter','latex');
%use 16 size font for Membership labels on graph.

%plotmf(Fuzzy_AYC_Ctl,'input',2)
plotmf(Fuzzy_AYC_Ctl,'input',2)
xlabel('$de_{\dot{r}}/dt$','FontSize',20,'Interpreter','latex');
ylabel('$Degree\, of\, Membership$','FontSize',20,'Interpreter','latex');
%use 16 size font for Membership labels on graph.

%plotmf(Fuzzy_AYC_Ctl,'output',1)
plotmf(Fuzzy_AYC_Ctl,'output',1)
%xlabel('$Normalized\, Output\, of\, Fuzzy\, Yaw\, Rate\, Error\, e_{\dot{r}}\, Controller$','FontSize',20,'Interpreter','latex');
xlabel('$Normalized\, Output\, of\, e_{\dot{r}}\, Controller$','FontSize',20,'Interpreter','latex');
ylabel('$Degree\, of\, Membership$','FontSize',20,'Interpreter','latex');
%use 16 size font for Membership labels on graph.
%saveas(figure1,'C:\Users\kschmutz\OneDrive\Thesis Paper and Other Works\Thesis LaTeX Rev1\Figures\Rev1\Fuzzy_Membership_Yaw_Output.jpg')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%plotmf(Fuzzy_Slip_Ctl_VLC,'input',1)
plotmf(Fuzzy_Slip_Ctl_VLC,'input',1)
xlabel('$d\lambda/dt$','FontSize',20,'Interpreter','latex');
ylabel('$Degree\, of\, Membership$','FontSize',20,'Interpreter','latex');
%use 16 size font for Membership labels on graph.

%plotmf(Fuzzy_Slip_Ctl_VLC,'input',2)
plotmf(Fuzzy_Slip_Ctl_VLC,'input',2)
xlabel('$\lambda$','FontSize',20,'Interpreter','latex');
ylabel('$Degree\, of\, Membership$','FontSize',20,'Interpreter','latex');
%use 16 size font for Membership labels on graph.

%plotmf(Fuzzy_Slip_Ctl_VLC,'output',1)
plotmf(Fuzzy_Slip_Ctl_VLC,'output',1)
xlabel('$Normalized\, Output\, of\, Fuzzy\, Slip\, \lambda\, Controller$','FontSize',20,'Interpreter','latex');
ylabel('$Degree\, of\, Membership$','FontSize',20,'Interpreter','latex');
%use 16 size font for Membership labels on graph.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%plotmf(Fuzzy_Slip_Ctl_ABS,'input',1)
plotmf(Fuzzy_Slip_Ctl_ABS,'input',1)
xlabel('$d\lambda/dt$','FontSize',20,'Interpreter','latex');
ylabel('$Degree\, of\, Membership$','FontSize',20,'Interpreter','latex');
%use 16 size font for Membership labels on graph.

%plotmf(Fuzzy_Slip_Ctl_ABS,'input',2)
plotmf(Fuzzy_Slip_Ctl_ABS,'input',2)
xlabel('$\lambda$','FontSize',20,'Interpreter','latex');
ylabel('$Degree\, of\, Membership$','FontSize',20,'Interpreter','latex');
%use 16 size font for Membership labels on graph.

%plotmf(Fuzzy_Slip_Ctl_ABS,'output',1)
plotmf(Fuzzy_Slip_Ctl_ABS,'output',1)
xlabel('$Normalized\, Output\, of\, Fuzzy\, Slip\, \lambda\, Controller$','FontSize',20,'Interpreter','latex');
ylabel('$Degree\, of\, Membership$','FontSize',20,'Interpreter','latex');
%use 16 size font for Membership labels on graph.