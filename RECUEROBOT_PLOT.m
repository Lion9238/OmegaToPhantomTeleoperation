clear;clc;
Master_data_Delay = load('CH1\MasterVelocity.txt','r');
Master_data_RealTime = load('CH1\DelayedMasterVelocity.txt','r');

%Slave_data_Delay = load('POPCON2\SlaveDelayTime.txt');
%Slave_data_RealTime = load('POPCON2\SlaveRealTime2.txt','r');

delayData = Master_data_Delay(:,2);
realData = Master_data_RealTime(:,2);

figure(1);
plot(delayData,'b');

hold on
plot(realData,'r');
legend('Delayed Data','Non Delayed Data');