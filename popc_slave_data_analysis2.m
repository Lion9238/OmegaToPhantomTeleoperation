% POPC simulation data analysis
clear;
clc;
close all;

Slave_data = load('slv_po_data_y.txt','r');

E_s_in = Slave_data(:,1);
E_s_out = -Slave_data(:,2); % E_s_out origin
E_s_out_modified = -Slave_data(:,3); % E_s_out modified
E_m_in = Slave_data(:,4);
E_pc = Slave_data(:,5);
Fs = Slave_data(:,6);
Vm_delayed = Slave_data(:,7);
Vs_modified = Slave_data(:,8);

Xm = zeros(length(Vm_delayed),1);
for i=2:length(Vm_delayed)
    Xm(i) = Xm(i-1)+Vm_delayed(i-1);
end

Xs = zeros(length(Vs_modified),1);
for i=2:length(Vs_modified)
    Xs(i) = Xs(i-1)+Vs_modified(i-1);
end

figure(1)
plot(E_m_in,'b');
hold on
plot(E_s_out_modified - E_pc,'r');
hold on
plot(E_m_in - E_s_out_modified + E_pc,'k');
hold on

plot(E_s_out,'k--');
% legend('Es origin','Es modified','Em in');
title('Slave POPC Data');
legend('Em in', 'Es out','Epc','E_s out origin');
xlabel('Time [sec]');
ylabel('Energy [Nmm]');

figure(2)
plot(Vm_delayed,'b');
hold on
plot(Vs_modified,'r');
legend('Vm','Vs');

           
figure(3)
plot(Xm,'b');
hold on
plot(Xs,'r');
legend('Xm','Xs');
xlabel('Time [sec]');
ylabel('X-axis Position [mm]');

figure(4)
subplot(2,1,1)
plot(Xs,'b');
subplot(2,1,2)
plot(Fs,'r');

figure(5) 
plot(E_s_out , 'b')
hold on
plot(E_s_out_modified , 'r')
legend('Es out origin' , 'Es out modified');
xlabel('Time[msec]');
ylabel('Energy[Nmm]');







