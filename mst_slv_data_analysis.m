close all;
clear;
clc;

%mst_data = load('mst_po_data_y.txt');
%slv_data = load('slv_po_data_y.txt');
mst_data = load('mst_data_x.txt');
slv_data = load('slv_data_x.txt');


s_position = slv_data(:,1); % Position of Phantom
s_position_prev = slv_data(:,2);
s_force = slv_data(:,3);
s_disp = slv_data(:,4);
s_vel = slv_data(:,5);

m_feedback = -mst_data(:,1);
m_pose = mst_data(:,2);
m_prevpose = mst_data(:,3);
m_vel = mst_data(:,4);
m_energy = mst_data(:,5);
[x y] = size(m_vel);
Xsd = zeros(x,y,'double');
energy_in= zeros(x,y,'double');
energy_out = zeros(x,y,'double');
for i = 2:x
    Xsd(i) = Xsd(i-1) + m_vel(i,1) * 0.001;
    energy_in(i) = energy_in(i-1) + s_force(i) * (m_pose(i)-s_position(i));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %if m_pose(i)-s_position(i) > 0.0
    %    energy_in(i) = energy_in(i-1) + s_force(i) * (m_pose(i)-s_position(i));
    %    energy_out(i) = energy_out(i-1);
    %else
    %    energy_in(i) = energy_in(i-1);
    %    energy_out(i) = energy_in(i-1) + s_force(i) * (m_pose(i)-s_position(i));
    %end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
%energy_in = -energy_in;
%energy_out = -energy_out;

figure(1)
plot(s_position,'b','LineWidth',2);
hold on
plot (m_pose,'r','LineWidth',2);
grid on
set(gca,'fontsize',24);
legend('slv pose','mst pose');
xlabel('\fontsize{32}time [ms]');
ylabel('\fontsize{32}position [mm]');
title('\fontsize{32}Position');

figure(2)
plot(s_force*10,'b','LineWidth',2);
hold on
plot ((m_pose - s_position)*0.1 - 0.06*(s_disp),'r','LineWidth',2);
grid on
set(gca,'fontsize',24);
legend('slv force','desired force');
xlabel('\fontsize{32}time [ms]');
ylabel('\fontsize{32}force [N]');
title('\fontsize{32}Force');

figure(3)
plot(s_force,'b','LineWidth',2);
hold on
plot ((m_pose - s_position) * 0.1,'r','LineWidth',2);
grid on
set(gca,'fontsize',24);
legend('slv force','slave position');
xlabel('\fontsize{32}time [ms]');
ylabel('\fontsize{32}force [N]');
title('\fontsize{32}Slave Force');



figure(4)
plot((m_pose-s_position),s_force,'k','LineWidth',1);
hold on
plot((m_pose - s_position),0.2*(m_pose - s_position),'r');
grid on

figure(5)
plot(energy_in,'r');
hold on
plot(energy_out,'b');
grid on
legend('energy in','energy out');
xlabel('\fontsize{16}time [ms]');
ylabel('\fontsize{16}Energy [J]');
title('\fontsize{32} Energy I/O');
