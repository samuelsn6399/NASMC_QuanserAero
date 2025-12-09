clc;clear;close all;

%% Model Parameters
T = 40; % [s] run time
DAQ_freq = 200; % [Hz] define the data acquisition frequency
DAQ_time = 1/DAQ_freq; % [s]

%% Configure Yaw and Pitch Commands
cmd_select = [0,2]; % [0,0] = ['smooth' transition to constant (polynomial curve fit), yaw only]
                    % [0,1] = ['smooth' transition to constant (polynomial curve fit), pitch only]
                    % [0,2] = ['smooth' transition to constant (polynomial curve fit), pitch and yaw]
                    % [1,0] = [harmonic oscilations, pitch only]
                    % [1,1] = [harmonic oscilations, yaw only]
                    % [1,2] = [harmonic oscilations, pitch and yaw]
Ap_cmd = pi/9; % [rad] amplitude of pitch command
Ay_cmd = pi/4; % [rad] amplitude of yaw command
Tp_cmd = 10; % [s] response time or period of response for pitch
Ty_cmd = 20; % [s] response time or period of response for yaw
harmonic_cycles = 2; % [] run simulation for n oscilation cycles
harmonic_cutoff_p = Tp_cmd*harmonic_cycles; % [s] sim time to stop oscilations
harmonic_cutoff_y = Ty_cmd*harmonic_cycles; % [s] sim time to stop oscilations
if cmd_select(1,1) == 0
    tp_cmd_final = [  Tp_cmd^4    Tp_cmd^5     Tp_cmd^6     Tp_cmd^7;
                    4*Tp_cmd^3  5*Tp_cmd^4   6*Tp_cmd^5   7*Tp_cmd^6;
                   12*Tp_cmd^2 20*Tp_cmd^3  30*Tp_cmd^4  42*Tp_cmd^5;
                   24*Tp_cmd   60*Tp_cmd^2 120*Tp_cmd^3 210*Tp_cmd^4];
    pitch_cmd_mag = [Ap_cmd;0;0;0];
    ap_cmd_coeff = tp_cmd_final\pitch_cmd_mag;
    ty_cmd_final = [  Ty_cmd^4    Ty_cmd^5     Ty_cmd^6     Ty_cmd^7;
                    4*Ty_cmd^3  5*Ty_cmd^4   6*Ty_cmd^5   7*Ty_cmd^6;
                   12*Ty_cmd^2 20*Ty_cmd^3  30*Ty_cmd^4  42*Ty_cmd^5;
                   24*Ty_cmd   60*Ty_cmd^2 120*Ty_cmd^3 210*Ty_cmd^4];
    yaw_cmd_mag = [Ay_cmd;0;0;0];
    ay_cmd_coeff = ty_cmd_final\yaw_cmd_mag;
else
    ap_cmd_coeff = [0;0;0;0];
    ay_cmd_coeff = [0;0;0;0];
end
a_cmd_coeff = [ap_cmd_coeff,ay_cmd_coeff];
T_cmd = [Tp_cmd, Ty_cmd];
A_cmd = [Ap_cmd, Ay_cmd];

%% Controller Gains
lambda_p = 2.5;       lambda_y = 2.5;
D_p = 0*1.1;        D_y = 0*1.1;
eta_p = 0.5;
eta_y = 0.5;
phi_para_p = 0.001;  phi_para_y = 0.001;
% lambda_p = 1;       lambda_y = 1;
% D_p = 0*1.1;        D_y = 0*1.1;
% eta_p = 0.5;
% eta_y = 0.5;
% phi_para_p = 1;  phi_para_y = 1;
% 
control_gains = [lambda_p lambda_y;
                 D_p D_y;
                 eta_p eta_y;
                 phi_para_p phi_para_y];


%% Turn-on Adaptation (0 == OFF)
% gamma0_p = 0.1;   gamma0_y = 0.1;
% gamma1_p = 0.1;   gamma1_y = 0.1;
% gamma2_p = 0.1;   gamma2_y = 0.1;
% gamma3_p = 0.1;   gamma3_y = 0.1;
% gamma4_p = 0.1;

gamma_y_gain = 100;
gamma_p_gain = 100;

gamma0_p = gamma_p_gain/sqrt(0.1);     gamma0_y = gamma_y_gain/sqrt(0.1);
gamma1_p = gamma_p_gain/sqrt(0.03);    gamma1_y = gamma_y_gain/sqrt(0.04);
gamma2_p = gamma_p_gain/sqrt(0.8);   
gamma3_p = gamma_p_gain/sqrt(500);     gamma2_y = gamma_y_gain/sqrt(400);   
gamma4_p = gamma_p_gain/sqrt(10000);   gamma3_y = gamma_y_gain/sqrt(10000);

% gamma0_p = 0;   gamma0_y = 0;
% gamma1_p = 0;   gamma1_y = 0;
% gamma2_p = 0;   gamma2_y = 0;
% gamma3_p = 0;   gamma3_y = 0;
% gamma4_p = 0;

gamma_p = [gamma0_p;gamma1_p;gamma2_p;gamma3_p;gamma4_p];
gamma_y = [gamma0_y;gamma1_y;gamma2_y;gamma3_y];

%% Pitch Parameters
g = 9.81; % [m/s^2]
m = 0.80+0.089+0.089; % [kg]
kFF = 1.25e-6; % [unitless]
dF = 0.2; % [m]
dcm = 0.00325; % [m]
Jp = 0.0219; % [kg-m^2]
kRT1 = 1e-5; % [unitless]
kRT2 = 1.25e-7; % [unitless]
Dp = 0.00711; % [V-s/rad]
kVF = 1/0.04222;
% kVF = 1; % turn off kv effects

%% Pitch haaaa
hp_hat = Jp/(kFF*dF*kVF^2);
ap1_hat = Dp/(kFF*dF*kVF^2);
ap2_hat = m*g*dcm/(kFF*dF*kVF^2);
ap3_hat = -kRT1/(kFF*dF*kVF^2);
ap4_hat = -kRT2/(kFF*dF*kVF^2);

%% Yaw Parameters
kFT1 = 1e-5; % [unitless]
kFT2 = 1.25e-7; % [unitless]
kRF = 1.25e-6; % [unitless]
dR = 0.2; %[m]
Dy = 0.0220; % [V-s/rad]
Jy = 0.0370; % [kg-m^2]
kVR = 1/0.04222; % [unitless]
% kVR = 1; % turn off kv effects

%% Yaw haaa
hy_hat = Jy/(kRF*dR*kVR^2);
ay1_hat = Dy/(kRF*dR*kVR^2);
ay2_hat = -kFT1/(kRF*dR*kVR^2);
ay3_hat = -kFT2/(kRF*dR*kVR^2);
% ay3_hat = 0;

%% Bundle Controller Params (Best Guess)
% params = [pitch params, yaw params]
params_aero = [kFF kRF; % NOT USED IN MODEL
               kRT1 kFT1;
               kRT2 kRT2;
               kVF kVR;
               dF dR;
               Jp Jy;
               Dp Dy;
               dcm 0;
               m 0;
               g 0];

paramsGuess_haaaa = [hp_hat hy_hat; % Best guess to initialize controller and adaptation
                    ap1_hat ay1_hat;
                    ap2_hat ay2_hat;
                    ap3_hat ay3_hat;
                    ap4_hat 0];

%% Inject Random Parameter Error
% used to adjust the plant model to "truth" values
% scenario: we do not understand our plant model, so we inject random
%       variation to the plant parameters to model this lack of understanding.
%       We take our best guess at the parameters to initialize the controller
paramErrorMax = 10;
paramErrorMin = 1/10;
% set seed for repeatable random errors
seedValue = 42; % pick any integer; change it for a new randomization pattern
rng(seedValue, 'twister');
paramsError_haaaa = paramErrorMin + (paramErrorMax-paramErrorMin).*rand(size(paramsGuess_haaaa));
% paramsError_haaaa = 1;

%% Bundle Plant Parameters (Truth)
paramsTruth_haaaa = paramsGuess_haaaa.*paramsError_haaaa; % true paramter values for plant model

%% Run Sim
out=sim("model_QuanserAero_AdaptiveV3_SN.slx");
% out=sim("model_QuanserAero_AdaptiveV3_21a_SN.mdl");

%% Parse Data
List=get(out);
for No=1:1:length(List)
    out(1,1).find(char(List(No,1)));
    assignin('base',char(List(No,1)),out(1,1).find(char(List(No,1))))
end
pitch(:,1) = rad2deg(states(1,1,:));
pitchd(:,1) = rad2deg(states(2,1,:));
yaw(:,1) = rad2deg(states(3,1,:));
yawd(:,1) = rad2deg(states(4,1,:));
% pitch(:,1) = rad2deg(states(:,1));
% pitchd(:,1) = rad2deg(states(:,2));
% yaw(:,1) = rad2deg(states(:,3));
% yawd(:,1) = rad2deg(states(:,4));
V_cmd = V;
rotor(:,1) = Omega(1,1,:);
rotor(:,2) = Omega(2,1,:);

%% Configure Plots
plot_type = 0; % 0=general viewing; 1=report (larger font)
if plot_type==0
    line_width = 0.75;
    text_size = 16;
    legend_size = 16;
    axis_size = 18;
    title_size = 24;
    sgtitle_size = 28;
else
    line_width = 1;
    text_size = 20;
    legend_size = 20;
    axis_size = 24;
    title_size = 32;
    sgtitle_size = 36;
end

%% Actual Vs Desired Plot (Combined)
figure('Name','PitchYawPosition');
hold on
plot(time,rad2deg(des(:,1)),'r--','LineWidth',line_width,'DisplayName','Desired Pitch, $\theta_d$')
plot(time,rad2deg(des(:,4)),'b--','LineWidth',line_width,'DisplayName','Desired Yaw, $\psi_d$')
plot(time,pitch(:,1),'r','LineWidth',line_width,'DisplayName','Actual Pitch, $\theta$')
plot(time,yaw(:,1),'b','LineWidth',line_width,'DisplayName','Actual Yaw, $\psi$')
hold off
title('Actual and Desired Yaw and Pitch Position','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('Pitch, $\theta$ / Yaw, $\psi$ (degrees)',...
       'Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on

%% Pitch Position
figure('Name','PitchPosition');
hold on
plot(time,rad2deg(des(:,1)),'r--','LineWidth',line_width,'DisplayName','Desired Pitch, $\theta_d$')
plot(time,pitch(:,1),'b','LineWidth',line_width,'DisplayName','Actual Pitch, $\theta$')
hold off
title('Pitch Position and Desired Position Over Time','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('Pitch, $\theta$ (degrees)','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on

%% Yaw Position
figure('Name','YawPosition');
hold on
plot(time,rad2deg(des(:,4)),'r--','LineWidth',line_width,'DisplayName','Desired Yaw, $\psi_d$')
plot(time,yaw(:,1),'b','LineWidth',line_width,'DisplayName','Actual Yaw, $\psi$')
hold off
title('Yaw Position and Desired Position Over Time','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('Yaw, $\psi$ (degrees)','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on

%% Pitch Error
figure('Name','PitchError');
plot(time,e_p(:,1),'r','LineWidth',line_width)
title('Pitch Error Over Time','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('Pitch Error, $\theta_d-\theta$ (rad)','Interpreter','Latex',FontSize=axis_size);
grid on
box on

%% Yaw Error
figure('Name','YawError');
plot(time,e_y(:,1),'r','LineWidth',line_width)
title('Yaw Error Over Time','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('Yaw Error, $\psi_d-\psi$ (rad)','Interpreter','Latex',FontSize=axis_size);
grid on
box on

%% Voltage Input
figure('Name','VoltageInput');
hold on
plot(time,V_cmd(:,1),'r','LineWidth',line_width,'DisplayName','Front Rotor Voltage, $V_F$')
plot(time,V_cmd(:,2),'b','LineWidth',line_width,'DisplayName','Rear Rotor Voltage, $V_R$')
hold off
title('Voltage Input Over Time','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('Voltage (V)','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on

%% Rotor Speed
figure('Name','RotorSpeed');
hold on
plot(time,rotor(:,1),'r','LineWidth',line_width,'DisplayName','Front Rotor Angular Speed, $\omega_F$')
plot(time,rotor(:,2),'b','LineWidth',line_width,'DisplayName','Rear Rotor Angular Speed, $\omega_R$')
hold off
title('Front and Rear Rotor Speed Over Time','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('Angular Speed, $\omega$ (rad/s)','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on

%% Pitch S Term
figure('Name','PitchSTerm')
hold on
plot(time,s_p(:,1),'r','LineWidth',line_width,'DisplayName','S Term')
plot(time,ones(1,length(time)).*phi_para_p,'r--','LineWidth',line_width,'DisplayName','Boundary Layer')
plot(time,ones(1,length(time)).*-phi_para_p,'r--','LineWidth',line_width,'DisplayName','')
hold off
title('Pitch S Term Over Time','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('S Term','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on
% axis([0 time(end) -1.25*phi_p 1.25*phi_p])

%% Yaw S Term
figure('Name','YawSTerm')
hold on
plot(time,s_y(:,1),'b','LineWidth',line_width,'DisplayName','S Term')
plot(time,ones(1,length(time)).*phi_para_y,'r--','LineWidth',line_width,'DisplayName','Boundary Layer')
plot(time,ones(1,length(time)).*-phi_para_y,'r--','LineWidth',line_width,'DisplayName','')
hold off
title('Yaw S Term Over Time','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('S Term','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on
% axis([0 time(end) -1.25*phi_y 1.25*phi_y])

%% Pitch Estimated Parameters
figure('Name','PitchEstimateParams')
sgtitle("Pitch Adaptation Over Time",'Interpreter','Latex',FontSize=sgtitle_size)
subplot(2,3,1)
hold on
plot(time,haaaa_hat_p(:,1),'b','LineWidth',line_width,'DisplayName','Adaptation')
% plot(time,ones(size(time)).*paramsTruth_haaaa(1,1),'b--','LineWidth',line_width,'DisplayName','Truth')
hold off
title('$h$ Parameter','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('$h$','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on
% axis([0 0.5 0 1.2*max([haaaa_hat_p(:,1);paramsTruth_haaaa(1,1)])])

subplot(2,3,2)
hold on
plot(time,haaaa_hat_p(:,2),'b','LineWidth',line_width,'DisplayName','Adaptation')
% plot(time,ones(size(time)).*paramsTruth_haaaa(2,1),'b--','LineWidth',line_width,'DisplayName','Truth')
hold off
title('$a_1$ Parameter','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('$a_1$','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on
% axis([0 0.5 0 1.2*max([haaaa_hat_p(:,2);paramsTruth_haaaa(2,1)])])

subplot(2,3,3)
hold on
plot(time,haaaa_hat_p(:,3),'b','LineWidth',line_width,'DisplayName','Adaptation')
% plot(time,ones(size(time)).*paramsTruth_haaaa(3,1),'b--','LineWidth',line_width,'DisplayName','Truth')
hold off
title('$a_2$ Parameter','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('$a_2$','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on
% axis([0 0.5 0 1.2*max([haaaa_hat_p(:,3);paramsTruth_haaaa(3,1)])])

subplot(2,3,4)
hold on
plot(time,haaaa_hat_p(:,4),'b','LineWidth',line_width,'DisplayName','Adaptation')
% plot(time,ones(size(time)).*paramsTruth_haaaa(4,1),'b--','LineWidth',line_width,'DisplayName','Truth')
hold off
title('$a_3$ Parameter','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex','Interpreter','Latex',FontSize=axis_size)
ylabel('$a_3$','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on
% axis([0 0.5 1.2*min([haaaa_hat_p(:,4);paramsTruth_haaaa(4,1)]) 0])

subplot(2,3,5)
hold on
plot(time,haaaa_hat_p(:,5),'b','LineWidth',line_width,'DisplayName','Adaptation')
% plot(time,ones(size(time)).*paramsTruth_haaaa(5,1),'b--','LineWidth',line_width,'DisplayName','Truth')
hold off
title('$a_4$ Parameter','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('$a_4$','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on
% axis([0 0.5 1.2*min([haaaa_hat_p(:,5);paramsTruth_haaaa(5,1)]) 0])

%% Yaw Estimated Parameters
figure('Name','YawEstimateParams')
sgtitle("Yaw Adaptation Over Time",'Interpreter','Latex',FontSize=sgtitle_size)
subplot(2,2,1)
hold on
plot(time,haaa_hat_y(:,1),'b','LineWidth',line_width,'DisplayName','Adaptation')
% plot(time,ones(size(time)).*paramsTruth_haaaa(1,2),'b--','LineWidth',line_width,'DisplayName','Truth')
hold off
title('$h$ Parameter','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('$h$','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on
% axis([0 0.5 0 1.2*max([haaa_hat_y(:,1);paramsTruth_haaaa(1,2)])])

subplot(2,2,2)
hold on
plot(time,haaa_hat_y(:,2),'b','LineWidth',line_width,'DisplayName','Adaptation')
% plot(time,ones(size(time)).*paramsTruth_haaaa(2,2),'b--','LineWidth',line_width,'DisplayName','Truth')
hold off
title('$a_1$ Parameter','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('$a_1$','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on
% axis([0 0.5 0 1.2*max([haaa_hat_y(:,2);paramsTruth_haaaa(2,2)])])

subplot(2,2,3)
hold on
plot(time,haaa_hat_y(:,3),'b','LineWidth',line_width,'DisplayName','Adaptation')
% plot(time,ones(size(time)).*paramsTruth_haaaa(3,2),'b--','LineWidth',line_width,'DisplayName','Truth')
hold off
title('$a_2$ Parameter','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex',FontSize=axis_size)
ylabel('$a_2$','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on
% axis([0 0.5 1.2*min([haaa_hat_y(:,3);paramsTruth_haaaa(3,2)]) 0])

subplot(2,2,4)
hold on
plot(time,haaa_hat_y(:,4),'b','LineWidth',line_width,'DisplayName','Adaptation')
% plot(time,ones(size(time)).*paramsTruth_haaaa(4,2),'b--','LineWidth',line_width,'DisplayName','Truth')
hold off
title('$a_3$ Parameter','Interpreter','Latex',FontSize=title_size)
xlabel('Time (s)','Interpreter','Latex','Interpreter','Latex',FontSize=axis_size)
ylabel('$a_3$','Interpreter','Latex',FontSize=axis_size)
legend('Interpreter','Latex','Location','best',FontSize=legend_size)
grid on
box on
% axis([0 0.5 1.2*min([haaa_hat_y(:,4);paramsTruth_haaaa(4,2)]) 0])

%% Error Terms
% Plot e, ed, and s from controller
% Plot expected e, ed, and s calculated from system params
% figure(5);clf(5);
% colororder(colors)
% subplot(2,3,1)
% plot(time,e_p.Data(:,1),'b','LineWidth',2)
% title('Pitch Error Vs Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$\theta_e - \theta_d (degrees)$','Interpreter','Latex')
% legend('$e_{\theta}$','Interpreter','Latex')
% % axis([0 time(end) -1 1])
% grid on
% box on
% 
% subplot(2,3,2)
% plot(time,ed_p.Data(:,1),'b','LineWidth',1)
% title('Pitch Derivative Error Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$d\theta_e - d\theta_d (degreees)$','Interpreter','Latex')
% legend('$de_{\theta}$','Interpreter','Latex')
% % axis([0 time(end) -1 1])
% grid on
% box on
% 
% subplot(2,3,3)
% plot(time,s_p(:,1),'b','LineWidth',1)
% title('Pitch S Term Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$S$','Interpreter','Latex')
% legend('$S_{\theta}$','Interpreter','Latex')
% % axis([0 time(end) -1 1])
% grid on
% box on
% 
% subplot(2,3,4)
% plot(time,e_y.Data(:,1),'b','LineWidth',2)
% title('Yaw Error Vs Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$\psi_e - \psi_d (degrees)$','Interpreter','Latex')
% legend('$e_{\psi}$','Interpreter','Latex')
% % axis([0 10 -1.5 0.5])
% grid on
% box on
% 
% subplot(2,3,5)
% plot(time,ed_y.Data(:,1),'b','LineWidth',1)
% title('Yaw Derivative Error Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$d\psi_e - d\psi_d (degreees)$','Interpreter','Latex')
% legend('$de_{\psi}$','Interpreter','Latex')
% % axis([0 time(end) -1 1])
% grid on
% box on
% 
% subplot(2,3,6)
% plot(time,s_y.Data(:,1),'b','LineWidth',1)
% title('Yaw S Term Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$S$','Interpreter','Latex')
% legend('$S_{\psi}$','Interpreter','Latex')
% % axis([0 time(end) -1 1])
% grid on
% box on
% 
% %% Parameter Estimation Terms
% % Plot a and h hat (estimates) and nominal (from known parameters) values
% figure(6);clf(6);
% colororder(colors)
% 
% subplot(2,5,1)
% hold on
% plot(time,haaaa_hat_p.Data(:,1),'b','LineWidth',1)
% plot(time,ones(size(time)).*paramsTruth_haaaa(1,1),'b--','LineWidth',1)
% hold off
% title('Pitch h Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$h$','Interpreter','Latex')
% legend('Adaptation $h$ Parameter', 'Truth $h$ Parameter','Interpreter','Latex')
% % axis([0 time(end) 0.25*paramsGuess_haaaa(1,1) 1.75*paramsGuess_haaaa(1,1)])
% grid on
% box on
% 
% subplot(2,5,2)
% hold on
% plot(time,haaaa_hat_p.Data(:,2),'r','LineWidth',1)
% plot(time,ones(size(time)).*paramsTruth_haaaa(2,1),'r--','LineWidth',1)
% hold off
% title('Pitch a1 Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_n$','Interpreter','Latex')
% legend('Adaptation $a1$ Parameter', ...
%         'Truth $a1$ Parameter', ...
%         'Interpreter','Latex')
% % axis([0 time(end) 0.25*paramsGuess_haaaa(2,1) 1.75*paramsGuess_haaaa(2,1)])
% grid on
% box on
% 
% subplot(2,5,3)
% hold on
% plot(time,haaaa_hat_p.Data(:,3),'m','LineWidth',1)
% plot(time,ones(size(time)).*paramsTruth_haaaa(3,1),'m--','LineWidth',1)
% hold off
% title('Pitch a2 Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_n$','Interpreter','Latex')
% legend('Adaptation $a2$ Parameter', ...
%        'Truth $a2$ Parameter', ...
%        'Interpreter','Latex')
% % axis([0 time(end) 0.25*paramsGuess_haaaa(3,1) 1.75*paramsGuess_haaaa(3,1)])
% grid on
% box on
% 
% subplot(2,5,4)
% hold on
% plot(time,haaaa_hat_p.Data(:,4),'b','LineWidth',1)
% plot(time,ones(size(time)).*paramsTruth_haaaa(4,1),'b--','LineWidth',1)
% hold off
% title('Pitch a3 Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_n$','Interpreter','Latex')
% legend('Adaptation $a3$ Parameter', ...
%        'Truth $a3$ Parameter', ...
%        'Interpreter','Latex')
% % axis([0 time(end) 1.75*paramsGuess_haaaa(4,1) 0.25*paramsGuess_haaaa(4,1)])
% grid on
% box on
% 
% subplot(2,5,5)
% hold on
% plot(time,haaaa_hat_p.Data(:,5),'g','LineWidth',1)
% plot(time,ones(size(time)).*paramsTruth_haaaa(5,1),'g--','LineWidth',1)
% hold off
% title('Pitch a4 Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_n$','Interpreter','Latex')
% legend('Adaptation $a4$ Parameter', ...
%        'Truth $a4$ Parameter', ...
%        'Interpreter','Latex')
% %axis([0 time(end) 1.75*paramsGuess_haaaa(5,1) 0.25*paramsGuess_haaaa(5,1)])
% grid on
% box on
% 
% subplot(2,5,6)
% hold on
% plot(time,haaa_hat_y.Data(:,1),'b','LineWidth',1)
% plot(time,ones(size(time)).*paramsTruth_haaaa(1,2),'b--','LineWidth',1)
% hold off
% title('Yaw h Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$h$','Interpreter','Latex')
% legend('Adaptation $h$ Parameter', 'Truth $h$ Parameter','Interpreter','Latex')
% % axis([0 time(end) 0.25*paramsGuess_haaaa(1,2) 1.75*paramsGuess_haaaa(1,2)])
% grid on
% box on
% 
% subplot(2,5,7)
% hold on
% plot(time,haaa_hat_y.Data(:,2),'r','LineWidth',1)
% plot(time,ones(size(time)).*paramsTruth_haaaa(2,2),'r--','LineWidth',1)
% hold off
% title('Yaw a1 Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_n$','Interpreter','Latex')
% legend('Adaptation $a1$ Parameter', ...
%        'Truth $a1$ Parameter', ...
%        'Interpreter','Latex')
% % axis([0 time(end) 0.25*paramsGuess_haaaa(2,2) 1.75*paramsGuess_haaaa(2,2)])
% grid on
% box on
% 
% subplot(2,5,8)
% hold on
% plot(time,haaa_hat_y.Data(:,3),'m','LineWidth',1)
% plot(time,ones(size(time)).*paramsTruth_haaaa(3,2),'m--','LineWidth',1)
% hold off
% title('Yaw a2 Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_n$','Interpreter','Latex')
% legend('Adaptation $a2$ Parameter', ...
%        'Truth $a2$ Parameter', ...
%        'Interpreter','Latex')
% % axis([0 time(end) 1.75*paramsGuess_haaaa(3,2) 0.25*paramsGuess_haaaa(3,2)])
% grid on
% box on
% 
% subplot(2,5,9)
% hold on
% plot(time,haaa_hat_y.Data(:,4),'b','LineWidth',1)
% plot(time,ones(size(time)).*paramsTruth_haaaa(4,2),'b--','LineWidth',1)
% hold off
% title('Yaw a3 Vs. Time')
% xlabel('$t$ (s)','Interpreter','Latex')
% ylabel('$a_n$','Interpreter','Latex')
% legend('Adaptation $a3$ Parameter', ...
%        'Truth $a3$ Parameter', ...
%        'Interpreter','Latex')
% %axis([0 time(end) 1.75*paramsGuess_haaaa(4,2) 0.25*paramsGuess_haaaa(4,2)])
% grid on
% box on
% 
% 
% %% Yaw Dependent Variables
% % f1,f2,f3 functions plotted as seen by the hat make, the controller, and
% % the plant
% figure(7);clf(7);
% colororder(colors)
% 
% subplot(3,1,1)
% hold on
% plot(time, fn_y_hatmaker.Data(:,1), 'b', 'LineWidth', 1)
% % plot(time, fn_y_controller.Data(:,1), 'r', 'LineWidth', 1)
% plot(time, fn_y_plant.Data(:,1), 'r--', 'LineWidth', 1)
% hold off
% title('Yaw Dependent Variable f1 Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('f1', 'Interpreter', 'Latex')
% legend('$hat maker$', 'plant', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.3 0.3])
% grid on
% box on
% 
% subplot(3,1,2)
% hold on
% plot(time, fn_y_hatmaker.Data(:,2), 'b', 'LineWidth', 1)
% % plot(time, fn_y_controller.Data(:,2), 'r', 'LineWidth', 1)
% plot(time, fn_y_plant.Data(:,2), 'r--', 'LineWidth', 1)
% hold off
% title('Yaw Dependent Variable f2 Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('f2', 'Interpreter', 'Latex')
% legend('$hat maker$', 'plant', 'Interpreter', 'Latex')
% % axis([0 time(end) -150 150])
% grid on
% box on
% 
% subplot(3,1,3)
% hold on
% plot(time, fn_y_hatmaker.Data(:,3), 'b', 'LineWidth', 1)
% % plot(time, fn_y_controller.Data(:,3), 'r', 'LineWidth', 1)
% plot(time, fn_y_plant.Data(:,3), 'r--', 'LineWidth', 1)
% hold off
% title('Yaw Dependent Variable f3 Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('f3', 'Interpreter', 'Latex')
% legend('$hat maker$', 'plant', 'Interpreter', 'Latex')
% % axis([0 time(end) -2.5E4 2.5E4])
% grid on
% box on
% 
% %% Yaw V and dV (Lyapunuv Function)
% figure(8);clf(8);
% colororder(colors)
% 
% subplot(2,1,1)
% plot(time, V_yaw.Data(:,1), 'b', 'LineWidth', 1)
% title('Yaw: V (Lyapunuv Function) Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('V', 'Interpreter', 'Latex')
% % axis([0 time(end) 0 5E3])
% grid on
% box on
% 
% subplot(2,1,2)
% plot(time, dV_yaw.Data(:,1), 'b', 'LineWidth', 1)
% title('Yaw: dV (Lyapunuv Function) Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('dV', 'Interpreter', 'Latex')
% % axis([0 time(end) -1 1])
% grid on
% box on
% 
% %% Yaw Lyapunuv Derivative Cancelation
% figure(9);clf(9);
% colororder(colors)
% 
% h_lyap_test_y = zeros(1,length(time));
% a1_lyap_test_y = zeros(1,length(time));
% a2_lyap_test_y = zeros(1,length(time));
% a3_lyap_test_y = zeros(1,length(time));
% s_lyap_test_y = zeros(1,length(time));
% 
% h_lyap_test_y(1,:) = lyap_test_y.Data(1,1,:);
% a1_lyap_test_y(1,:) = lyap_test_y.Data(2,1,:);
% a2_lyap_test_y(1,:) = lyap_test_y.Data(3,1,:);
% a3_lyap_test_y(1,:) = lyap_test_y.Data(4,1,:);
% s_lyap_test_y(1,:) = lyap_test_y.Data(5,1,:);
% 
% subplot(5,1,1)
% plot(time, h_lyap_test_y, 'b', 'LineWidth', 1)
% title('Yaw: h Lyapunuv Cancelation Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('h dV component', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.1 0.1])
% grid on
% box on
% 
% subplot(5,1,2)
% plot(time, a1_lyap_test_y, 'b', 'LineWidth', 1)
% title('Yaw: a1 Lyapunuv Cancelation Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('a1 dV component', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.1 0.1])
% grid on
% box on
% 
% subplot(5,1,3)
% plot(time, a2_lyap_test_y, 'b', 'LineWidth', 1)
% title('Yaw: a2 Lyapunuv Cancelation Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('a2 dV component', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.1 0.1])
% grid on
% box on
% 
% subplot(5,1,4)
% plot(time, a3_lyap_test_y, 'b', 'LineWidth', 1)
% title('Yaw: a3 Lyapunuv Cancelation Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('a3 dV component', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.1 0.1])
% grid on
% box on
% 
% subplot(5,1,5)
% plot(time, s_lyap_test_y, 'b', 'LineWidth', 1)
% title('Yaw: s Lyapunuv Cancelation Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('s dV component', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.1 0.1])
% grid on
% box on
% 
% %% Pitch V and dV (Lyapunuv Function)
% figure(10);clf(10);
% colororder(colors)
% 
% subplot(2,1,1)
% plot(time, V_pitch.Data(:,1), 'b', 'LineWidth', 1)
% title('Pitch: V (Lyapunuv Function) Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('V', 'Interpreter', 'Latex')
% % axis([0 time(end) 0 5E3])
% grid on
% box on
% 
% subplot(2,1,2)
% plot(time, dV_pitch.Data(:,1), 'b', 'LineWidth', 1)
% title('Pitch: dV (Lyapunuv Function) Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('dV', 'Interpreter', 'Latex')
% % axis([0 time(end) -1 1])
% grid on
% box on
% 
% %% Pitch Lyapunuv Derivative Cancelation
% figure(11);clf(11);
% colororder(colors)
% 
% h_lyap_test_p = zeros(1,length(time));
% a1_lyap_test_p = zeros(1,length(time));
% a2_lyap_test_p = zeros(1,length(time));
% a3_lyap_test_p = zeros(1,length(time));
% a4_lyap_test_p = zeros(1,length(time));
% s_lyap_test_p = zeros(1,length(time));
% 
% h_lyap_test_p = lyap_test_p.Data(1,1,:);
% a1_lyap_test_p(1,:) = lyap_test_p.Data(2,1,:);
% a2_lyap_test_p(1,:) = lyap_test_p.Data(3,1,:);
% a3_lyap_test_p(1,:) = lyap_test_p.Data(4,1,:);
% a4_lyap_test_p(1,:) = lyap_test_p.Data(5,1,:);
% s_lyap_test_p = lyap_test_p.Data(6,1,:);
% 
% subplot(6,1,1)
% plot(time, a1_lyap_test_p, 'b', 'LineWidth', 1)
% title('Pitch: h Lyapunuv Cancelation Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('h dV component', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.1 0.1])
% grid on
% box on
% 
% subplot(6,1,2)
% plot(time, a1_lyap_test_p, 'b', 'LineWidth', 1)
% title('Pitch: a1 Lyapunuv Cancelation Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('a1 dV component', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.1 0.1])
% grid on
% box on
% 
% subplot(6,1,3)
% plot(time, a2_lyap_test_p, 'b', 'LineWidth', 1)
% title('Pitch: a2 Lyapunuv Cancelation Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('a2 dV component', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.1 0.1])
% grid on
% box on
% 
% subplot(6,1,4)
% plot(time, a3_lyap_test_p, 'b', 'LineWidth', 1)
% title('Pitch: a3 Lyapunuv Cancelation Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('a3 dV component', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.1 0.1])
% grid on
% box on
% 
% subplot(6,1,5)
% plot(time, a4_lyap_test_p, 'b', 'LineWidth', 1)
% title('Pitch: a4 Lyapunuv Cancelation Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('a4 dV component', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.1 0.1])
% grid on
% box on
% 
% subplot(6,1,6)
% plot(time, a1_lyap_test_p, 'b', 'LineWidth', 1)
% title('Pitch: s Lyapunuv Cancelation Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('s dV component', 'Interpreter', 'Latex')
% % axis([0 time(end) -0.1 0.1])
% grid on
% box on
% 
% %% Yaw sat Parameter investigation
% % plot const. phi parameter and time varying s on one plot
% % plot sat output on another graph
% figure(12); clf(12);
% colororder(colors)
% 
% subplot(2,1,1)
% hold on
% plot(time, ones(1,length(time)).*phi_para_y, 'r--', 'LineWidth', 1)
% plot(time, ones(1,length(time)).*-phi_para_y, 'r--', 'LineWidth', 1)
% plot(time, s_y.Data(:,1), 'b', 'LineWidth', 1)
% hold off
% title('Yaw s term with phi Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('s', 'Interpreter', 'Latex')
% % axis([0 time(end) -1.25 1.25])
% grid on
% box on
% 
% subplot(2,1,2)
% plot(time,sat_y.Data(:,1),'b', 'LineWidth', 1)
% title('Yaw sat Vs. Time')
% xlabel('$t$ (s)', 'Interpreter', 'Latex')
% ylabel('sat', 'Interpreter', 'Latex')
% % axis([0 time(end) -1.25 1.25])
% grid on
% box on