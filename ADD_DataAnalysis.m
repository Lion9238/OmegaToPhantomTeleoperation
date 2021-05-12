clear;
clc;
close all;

LFTsensor = load('CH1/ForceTorque.txt','r');
LMstVelocity = load('CH1/MasterVelocity.txt','r');
LMstPosition = load('CH1/MasterPosition.txt','r');

RFTsensor = load('CH2/ForceTorque.txt','r');
RMstVelocity = load('CH2/MasterVelocity.txt','r');
RMstPosition = load('CH2/MasterPosition.txt','r');

Mode = 1;

%% Mode 1
% Left Arm
if Mode == 1
close all;

figure(1)
plot(LFTsensor(:,1),'b','LineWidth',2);
hold on
plot(LFTsensor(:,2),'r','LineWidth',2);
hold on
plot(LFTsensor(:,3),'k','LineWidth',2);
hold on
legend('X Axis','Y Axis','Z Axis');
title('\fontsize{24}Left Arm FTsensor Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}FTsensor Data');
grid on

figure(2)
plot(LMstVelocity(:,1),'b','LineWidth',2);
hold on
plot(LMstVelocity(:,2),'r','LineWidth',2);
hold on
plot(LMstVelocity(:,3),'k','LineWidth',2);
hold on
legend('X Axis','Y Axis','Z Axis');
title('\fontsize{24}Left Arm Master Velocity Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstVelocity Data [mm/s]');
grid on

figure(3)
plot(LMstPosition(:,1),'b','LineWidth',2);
hold on
plot(LMstPosition(:,2),'r','LineWidth',2);
hold on
plot(LMstPosition(:,3),'k','LineWidth',2);
hold on
legend('X Axis','Y Axis','Z Axis');
title('\fontsize{24}Left Arm Master Position Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstPosition Data [mm]');
grid on

% Right Arm
figure(4)
plot(RFTsensor(:,1),'b','LineWidth',2);
hold on
plot(RFTsensor(:,2),'r','LineWidth',2);
hold on
plot(RFTsensor(:,3),'k','LineWidth',2);
hold on
legend('X Axis','Y Axis','Z Axis');
title('\fontsize{24}Right Arm FTsensor Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}FTsensor Data');
grid on

figure(5)
plot(RMstVelocity(:,1),'b','LineWidth',2);
hold on
plot(RMstVelocity(:,2),'r','LineWidth',2);
hold on
plot(RMstVelocity(:,3),'k','LineWidth',2);
hold on
legend('X Axis','Y Axis','Z Axis');
title('\fontsize{24}Right Arm Master Velocity Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstVelocity Data [mm/s]');
grid on

figure(6)
plot(RMstPosition(:,1),'b','LineWidth',2);
hold on
plot(RMstPosition(:,2),'r','LineWidth',2);
hold on
plot(RMstPosition(:,3),'k','LineWidth',2);
hold on
legend('X Axis','Y Axis','Z Axis');
title('\fontsize{24}Right Arm Master Position Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstPosition Data [mm]');
grid on



%% mode 2
elseif Mode== 2
close all;
% Left Arm
figure(1)
subplot(3,1,1);
plot(LFTsensor(:,1),'b','LineWidth',2);
title('\fontsize{24}Left Arm FTsensor Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}FTsensor Data');
legend('X Axis');
grid on
subplot(3,1,2);
plot(LFTsensor(:,2),'r','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}FTsensor Data');
legend('Y Axis');
grid on
subplot(3,1,3);
plot(LFTsensor(:,3),'k','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}FTsensor Data');
legend('Z Axis');
grid on


figure(2)
subplot(3,1,1);
plot(LMstVelocity(:,1),'b','LineWidth',2);
title('\fontsize{24}Left Arm Master Velocity Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstVelocity Data [mm/s]');
legend('X Axis');
grid on
subplot(3,1,2);
plot(LMstVelocity(:,2),'r','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstVelocity Data [mm/s]');
legend('Y Axis');
grid on
subplot(3,1,3);
plot(LMstVelocity(:,3),'k','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstVelocity Data [mm/s]');
legend('Z Axis');
grid on


figure(3)
subplot(3,1,1);
plot(LMstPosition(:,1),'b','LineWidth',2);
title('\fontsize{24}Left Arm Master Position Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstPosition Data [mm]');
legend('X Axis');
grid on
subplot(3,1,2);
plot(LMstPosition(:,2),'r','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstPosition Data [mm]');
legend('Y Axis');
grid on
subplot(3,1,3);
plot(LMstPosition(:,3),'k','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstPosition Data [mm]');
legend('Z Axis');
grid on

% Right Arm
figure(4)
subplot(3,1,1);
plot(RFTsensor(:,1),'b','LineWidth',2);
title('\fontsize{24}Right Arm FTsensor Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}FTsensor Data');
legend('X Axis');
grid on
subplot(3,1,2);
plot(RFTsensor(:,2),'r','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}FTsensor Data');
legend('Y Axis');
grid on
subplot(3,1,3);
plot(RFTsensor(:,3),'k','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}FTsensor Data');
legend('Z Axis');
grid on


figure(5)
subplot(3,1,1);
plot(RMstVelocity(:,1),'b','LineWidth',2);
title('\fontsize{24}Right Arm Master Velocity Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstVelocity Data [mm/s]');
legend('X Axis');
grid on
subplot(3,1,2);
plot(RMstVelocity(:,2),'r','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstVelocity Data [mm/s]');
legend('Y Axis');
grid on
subplot(3,1,3);
plot(RMstVelocity(:,3),'k','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstVelocity Data [mm/s]');
legend('Z Axis');
grid on


figure(6)
subplot(3,1,1);
plot(RMstPosition(:,1),'b','LineWidth',2);
title('\fontsize{24}Right Arm Master Position Data');
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstPosition Data [mm]');
legend('X Axis');
grid on
subplot(3,1,2);
plot(RMstPosition(:,2),'r','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstPosition Data [mm]');
legend('Y Axis');
grid on
subplot(3,1,3);
plot(RMstPosition(:,3),'k','LineWidth',2);
xlabel('\fontsize{16}Time [sec]');
ylabel('\fontsize{16}MstPosition Data [mm]');
legend('Z Axis');
grid on
end