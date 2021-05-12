% POPC simulation data analysis
% Including F_vms(virtual mass spring based force)

clear;
close all;


Master_data = load('mst_po_data_y.txt','r');

E_m_in = Master_data(:,1);
E_m_out = -Master_data(:,2);
E_m_out2 = -Master_data(:,3);
E_s_in = Master_data(:,4);
E_m_pc = Master_data(:,5);
Fs_delayed = Master_data(:,6);
Fm_modified = Master_data(:,7);
Fm_vms = Master_data(:,8);
Vm = Master_data(:,9);

%for i=1:length(Fm_vms)
%    if Fm_vms(i) < 0
%        Fm_vms(i) = 0;
%    end
%end


Xm = zeros(length(Vm),1);
for i=2:length(Vm)
    Xm(i) = Xm(i-1)+Vm(i-1);
end


figure(1)

plot(E_s_in,'b');
hold on
plot(E_s_in - E_m_out2 + E_m_pc,'k');
hold on
plot(E_m_out2 - E_m_pc,'r');
%plot(E_m_out2 - E_m_pc,'r');
% legend('Es in','Em origin','Em modified');
grid on
title('Master POPC Data');
legend('Es in','Em pc','Em out');
xlabel('Time [sec]');
ylabel('Energy [Nmm]');

figure(2)
plot(Fs_delayed,'b');
hold on
plot(Fm_modified,'r');
hold on
plot(Fm_vms,'k');
 legend('Fs','F modified','F vms');
%legend('Fs','Fs modified');
xlabel('Time [sec]');
ylabel('Force [N]');

figure(3)
subplot(2,1,1)
plot(E_m_out,'b');
grid on
hold on
plot(E_m_out2,'r');
plot(E_s_in,'g');
legend('Em origin','Em modified','Es in');

subplot(2,1,2)
plot(Fs_delayed,'b');
grid on
hold on
plot(Fm_modified,'r');
legend('Fs','F modified');

figure(4)
plot(Vm,'b');

figure(5)
plot(Xm,'b');




