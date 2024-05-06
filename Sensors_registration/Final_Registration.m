%Test for IMU-EM alignement based on Chardonnens method 
clear all
close all

%% Load experimental data
%Data acquired with XSens and Aurora fixed one with respect to the other
%and performing rotations of approximately 360° around 3 orthogonal axis
A = load('AuroraQuat_19_05_2.mat');
A = A.ans;
X = load('XSensQuat_19_05_2.mat');
X = X.ans;
X2 = load('XSensModQuat_19_05_2.mat');
X2 = X2.ans;
V = load('XSensAngVel_19_05_2.mat');
V = V.ans;

%% Data preprocessing
%Search for a time zero to align the Aurora and XSens data based on
%real sampling time of Aurora
ref = A(2:5,1);
index = 1;
different = true;
while(different)
    index = index+1;
    comp = A(2:5,index);
    if(~isequal(ref,comp))
        different = false;
    end   
end
A_aligned = A(2:5,index:end);
X_aligned = X(2:5,index:end);
X2_aligned = X2(2:5,index:end);
V_aligned = V(2:end,index:end);
%At this point we have aligned data on the real sampling time of Aurora
%Anyway, the two vectors contain data at 100Hz but these cannot be use to
%compute angular velocities on Aurora because of the sample-and-hold
%upsampling strategy to pass from 40 to 100Hz
%To deal with the different sampling frequencies, let's just downsample
%both the sensor data to 20Hz
A_downsampled = [downsample(A_aligned(1,:),5);downsample(A_aligned(2,:),5);downsample(A_aligned(3,:),5);downsample(A_aligned(4,:),5)];
X_downsampled = [downsample(X_aligned(1,:),5);downsample(X_aligned(2,:),5);downsample(X_aligned(3,:),5);downsample(X_aligned(4,:),5)];
X2_downsampled = [downsample(X2_aligned(1,:),5);downsample(X2_aligned(2,:),5);downsample(X2_aligned(3,:),5);downsample(X2_aligned(4,:),5)];
V_downsampled = [downsample(V_aligned(1,:),5);downsample(V_aligned(2,:),5);downsample(V_aligned(3,:),5)];

%% Compute angular velocities in local RF from quaternions
dt = 1/20;      %we have downsampled data at 20Hz
dt = dt*5;

omega_IMU = quaternion2AngularVelocityL(X_downsampled,dt);
omega_IMU2 = quaternion2AngularVelocityL(X2_downsampled,dt);
omega_EM = quaternion2AngularVelocityL(A_downsampled,dt);

%% Apply Chardonnens method and get rotation matrix from IMU to COIL
%In reality this should be from COIL to IMU, based on Chardonnens inputs,
%but the order in Chardonnens input is wrong
%Chardonnens method using original XSens quaternion
IMU_R_COIL = ChardonnensMethod(omega_IMU,omega_EM,dt);           %we are in local frame so we consider inverted input for Chardonnens
IMU_q_COIL = rotm2quat(IMU_R_COIL);
%Chardonnens method using modified XSens quaternion
IMU_R2_COIL = ChardonnensMethod(omega_IMU2,omega_EM,dt);
IMU_q2_COIL = rotm2quat(IMU_R2_COIL);

%% Compute angular velocities in inertial RF from quaternions
dt = 1/20;      %we have downsampled data at 20Hz
dt = dt*5;

omega_IMU = quaternion2AngularVelocityI(X_downsampled,dt);
omega_IMU2 = quaternion2AngularVelocityI(X2_downsampled,dt);
omega_EM = quaternion2AngularVelocityI(A_downsampled,dt);

%% Apply Chardonnens method and get rotation matrix from FG to ENU (we invert Chardonnens inputs since we are in inertial frames)
%Looking at acceleration component, since [1,0,0] is mapped into [0,0,1],
%as gravity should do from FG to ENU RF
dt = dt/5;
%Chardonnens method using original XSens quaternion
ENU_R_FG = ChardonnensMethod(omega_EM,omega_IMU,dt);           %V_downsampled(:,1:end-1)
ENU_q_FG = rotm2quat(ENU_R_FG);
%Chardonnens method using modified XSens quaternion
ENU_R2_FG = ChardonnensMethod(omega_EM,omega_IMU2,dt);
ENU_q2_FG = rotm2quat(ENU_R2_FG);

%% Alignement test
%new_X is computed as the concatenation of rotations, it should be
%ENU_R_FG * IMU_R_ENU * COIL_R_IMU = COIL_R_FG
%where: - ENU_R_FG is the rotation from FG to ENU, computed with
%       Chardonnens on inertial;
%       - IMU_R_ENU is the rotation from ENU to IMU, as given by XSens
%       sensor;
%       - COIL_R_IMU is the rotation from IMU to COIL, computed with
%       Chardonnens on local;
%       - COIL_R_FG is the rotation from FG to COIL, computed as the
%       concatenation of such rotations and should be comparable with the
%       one provided by Aurora.
new_X = quatmultiply(ENU_q2_FG,quatmultiply(X2_downsampled',IMU_q2_COIL));
eul_EM = (quat2eul(A_downsampled'));
eul_IMU = (quat2eul(X_downsampled'));
eul_IMU_referred = (quat2eul(new_X));

error = eul_IMU_referred - eul_EM;
error_quat = new_X-A_downsampled';

figure
subplot(3,1,1)
plot(error(:,1)*(180/pi))
grid on
ylabel('[°]')
xlabel('sample')
title('Registration error on x')
subplot(3,1,2)
plot(error(:,2)*(180/pi))
grid on
ylabel('[°]')
xlabel('sample')
title('Registration error on y')
subplot(3,1,3)
plot(error(:,3)*(180/pi))
grid on
ylabel('[°]')
xlabel('sample')
title('Registration error on z')

figure
boxplot(error*(180/pi),'Labels', {'Z','Y','X'})
grid on
ylabel('[°]')
title('Registration error')

figure
subplot(4,1,1)
plot(error_quat(:,1))
grid on
ylabel('[]')
xlabel('sample')
title('Registration error on w')
subplot(4,1,2)
plot(error_quat(:,2))
grid on
ylabel('[]')
xlabel('sample')
title('Registration error on x')
subplot(4,1,3)
plot(error_quat(:,3))
grid on
ylabel('[]')
xlabel('sample')
title('Registration error on y')
subplot(4,1,4)
plot(error_quat(:,4))
grid on
ylabel('[]')
xlabel('sample')
title('Registration error on z')

figure
subplot(3,1,1)
plot(eul_EM(:,1),'k','Linewidth',1)
hold on
grid on
plot(eul_IMU(:,1),'b','Linewidth',1)
plot(eul_IMU_referred(:,1),'y--','Linewidth',1)
xlabel('sample')
ylabel('[rad]')
legend('EM','IMU before','IMU aligned')
subplot(3,1,2)
plot(eul_EM(:,2),'k','Linewidth',1)
hold on
grid on
plot(eul_IMU(:,2),'b','Linewidth',1)
plot(eul_IMU_referred(:,2),'y--','Linewidth',1)
xlabel('sample')
ylabel('[rad]')
subplot(3,1,3)
plot(eul_EM(:,3),'k','Linewidth',1)
hold on
grid on
plot(eul_IMU(:,3),'b','Linewidth',1)
plot(eul_IMU_referred(:,3),'y--','Linewidth',1)
xlabel('sample')
ylabel('[rad]')
sgtitle('Alignement with original XSens quaternion')

figure
subplot(4,1,1)
plot(A_downsampled(1,:),'k','Linewidth',1)
hold on
grid on
plot(X_downsampled(1,:),'b','Linewidth',1)
plot(new_X(:,1),'y--','Linewidth',1)
legend('EM','IMU before','IMU aligned')
subplot(4,1,2)
plot(A_downsampled(2,:),'k','Linewidth',1)
hold on
grid on
plot(X_downsampled(2,:),'b','Linewidth',1)
plot(new_X(:,2),'y--','Linewidth',1)
subplot(4,1,3)
plot(A_downsampled(3,:),'k','Linewidth',1)
hold on
grid on
plot(X_downsampled(3,:),'b','Linewidth',1)
plot(new_X(:,3),'y--','Linewidth',1)
subplot(4,1,4)
plot(A_downsampled(4,:),'k','Linewidth',1)
hold on
grid on
plot(X_downsampled(4,:),'b','Linewidth',1)
plot(new_X(:,4),'y--','Linewidth',1)
sgtitle('Alignement with original XSens quaternion')

%% Test for free-acceleration computation
g = [9.81;0;0];                                  %gravity in Aurora RF based on pose of PFG
Acc = load('XSensAcc_19_05_2.mat');              %measured acceleration from XSens in sensor local RF
Acc = Acc.ans;
Acc_aligned = Acc(2:4,index:end);
Acc_downsampled = [downsample(Acc_aligned(1,:),5);downsample(Acc_aligned(2,:),5);downsample(Acc_aligned(3,:),5)];
Fac = load('XSensFreeAcc_19_05_2.mat');          %free acceleration provided by internal Xsens algorithm in ENU
Fac = Fac.ans;
Fac_aligned = Fac(2:4,index:end);
Fac_downsampled = [downsample(Fac_aligned(1,:),5);downsample(Fac_aligned(2,:),5);downsample(Fac_aligned(3,:),5)];
Fac_registered = [];

%The acceleration in the FG inertial RF can be computed from the acceleration in IMU
%local RF considering two different paths:
%- from IMU to COIL and then from COIL to FG, as in Acc3;
%- from IMU to ENU and then from ENU to FG, as in Acc2.
Acc2 = [];
Acc3 = [];
for i=1:length(Acc_downsampled)
    %Compute overall quaternion to pass from IMU to FG, this is the
    %composition of IMU to COIL and COIL to FG
    quat_COIL = quatmultiply(IMU_q_COIL,quatinv(A_downsampled(:,i)'));
    %The inertial sensor does not measure acceleration, but reaction force
    %so to have acceleration in the common sense we need to change sign
    Acc3 = [Acc3,quaternionRotLocal(quat_COIL',-Acc_downsampled(:,i))];
    %Compute overall quaternion to pass from IMU to FG, this is the
    %composition of IMU to ENU and ENU to FG
    quat_ENU = quatmultiply(quatinv(X_downsampled(:,i)'),quatinv(ENU_q_FG));
    %The inertial sensor does not measure acceleration, but reaction force
    %so to have acceleration in the common sense we need to change sign
    Acc2 = [Acc2,quaternionRotLocal(quat_ENU',-Acc_downsampled(:,i))];
    %The XSens Free Acceleration is provided in the ENU RF, so we need to
    %map it to the FG one
    Fac_registered = [Fac_registered,quaternionRot(quatinv(ENU_q_FG)',Fac_downsampled(:,i))];
end

figure
subplot(3,1,1)
plot(Acc_downsampled(1,:),'b')
hold on
grid on
plot(Acc2(1,:),'r')
plot(Acc3(1,:),'g')
xlabel('sample')
ylabel('[m/s^2]')
title('Acceleration on x')
legend('Local acc IMU','Acceleration in FG frame passing from ENU','Acceleration in FG frame directly')
subplot(3,1,2)
plot(Acc_downsampled(2,:),'b')
hold on
grid on
plot(Acc2(2,:),'r')
plot(Acc3(2,:),'g')
xlabel('sample')
ylabel('[m/s^2]')
title('Acceleration on y')
subplot(3,1,3)
plot(Acc_downsampled(3,:),'b')
hold on
grid on
plot(Acc2(3,:),'r')
plot(Acc3(3,:),'g')
xlabel('sample')
ylabel('[m/s^2]')
title('Acceleration on z')

Fac_mine = Acc3 - ones(size(Acc2)).*g;
error_acc = abs(Fac_mine-Fac_registered);
figure
subplot(3,1,1)
plot(Fac_registered(1,:),'b')
hold on
grid on
plot(Fac_mine(1,:),'r')
xlabel('sample')
ylabel('[m/s^2]')
title('Acceleration on x')
legend('XSens free acc','Mine free acc')
subplot(3,1,2)
plot(Fac_registered(2,:),'b')
hold on
grid on
plot(Fac_mine(2,:),'r')
xlabel('sample')
ylabel('[m/s^2]')
title('Acceleration on y')
subplot(3,1,3)
plot(Fac_registered(3,:),'b')
hold on
grid on
plot(Fac_mine(3,:),'r')
xlabel('sample')
ylabel('[m/s^2]')
title('Acceleration on z')
figure
subplot(3,1,1)
plot(error_acc(1,:))
grid on
xlabel('sample')
ylabel('[m/s^2]')
title('Error on x Free acceleration')
subplot(3,1,2)
plot(error_acc(2,:))
grid on
xlabel('sample')
ylabel('[m/s^2]')
title('Error on y Free acceleration')
subplot(3,1,3)
plot(error_acc(3,:))
grid on
xlabel('sample')
ylabel('[m/s^2]')
title('Error on z Free acceleration')

figure
i = 1;
subplot(3,1,1)
plot(i,Acc_downsampled(1,i),'b.')
hold on
grid on
plot(i,Acc2(1,i),'r.')
plot(i,Acc3(1,i),'g.')
xlim([0,1000])
ylim([-20,20])
xlabel('sample')
ylabel('[m/s^2]')
title('Acceleration on x')
legend('Local acc IMU','Acceleration in FG frame passing from ENU','Acceleration in FG frame directly')
subplot(3,1,2)
plot(i,Acc_downsampled(2,i),'b.')
hold on
grid on
plot(i,Acc2(2,i),'r.')
plot(i,Acc3(2,i),'g.')
xlim([0,1000])
ylim([-20,20])
xlabel('sample')
ylabel('[m/s^2]')
title('Acceleration on y')
subplot(3,1,3)
plot(i,Acc_downsampled(3,i),'b.')
hold on
grid on
plot(i,Acc2(3,i),'r.')
plot(i,Acc3(3,i),'g.')
xlim([0,1000])
ylim([-20,20])
xlabel('sample')
ylabel('[m/s^2]')
title('Acceleration on z')
for i = 2:1000%length(eul_EM(:,1))
    subplot(3,1,1)
    plot(i,Acc_downsampled(1,i),'b.','HandleVisibility','off')
    hold on
    grid on
    plot(i,Acc2(1,i),'r.','HandleVisibility','off')
    plot(i,Acc3(1,i),'g.','HandleVisibility','off')
    subplot(3,1,2)
    plot(i,Acc_downsampled(2,i),'b.','HandleVisibility','off')
    hold on
    grid on
    plot(i,Acc2(2,i),'r.','HandleVisibility','off')
    plot(i,Acc3(2,i),'g.','HandleVisibility','off')
    subplot(3,1,3)
    plot(i,Acc_downsampled(3,i),'b.','HandleVisibility','off')
    hold on
    grid on
    plot(i,Acc2(3,i),'r.','HandleVisibility','off')
    plot(i,Acc3(3,i),'g.','HandleVisibility','off')
    if(i == 2)
        %pause(10)
    else
        %pause(0.000000001)
    end
end

figure
i = 1;
plot(i,eul_EM(i,1),'ko','Linewidth',1)
hold on
grid on
plot(i,eul_IMU(i,1),'bo','Linewidth',1)
plot(i,eul_IMU_referred(i,1),'yo','Linewidth',1)
xlim([0,length(eul_EM(:,1))])
ylim([-4,4])
xlabel('sample')
ylabel('[rad]')
% legend('EM','IMU before','IMU aligned')
for i = 2:length(eul_EM(:,1))
    plot(i,eul_EM(i,1),'k.-');%,'k','Linewidth',1)
    hold on
    grid on
    plot(i,eul_IMU(i,1),'b.-');%,'b','Linewidth',1)
    plot(i,eul_IMU_referred(i,1),'y.-');%,'y--','Linewidth',1)
    %pause(0.001)
end