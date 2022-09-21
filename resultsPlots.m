clear; clc; close all;

%% Load data
load('2022-09-21_14-33 (constrained 4.5).mat');
const4_5 = distCum;
clearvars -except const4_5

load('2022-09-21_16-41 (soc_constant).mat');
socConst = distCum;
clearvars -except const4_5 socConst

load('2022-09-16_07-33 (MPC_Test1).mat');
mpc = distCum;
clearvars -except const4_5 socConst mpc

load('2022-09-19_13-30 (4.5 const, no battery constraint).mat');
maxSpeed = distCum;
clearvars -except const4_5 socConst mpc maxSpeed

load('2022-09-20_18-29 (mpc_v2).mat');
mpc_v2 = distCum;
clearvars -except const4_5 socConst mpc maxSpeed mpc_v2



%% Plot Data
hold on;
plot(const4_5, '-r', 'DisplayName', 'Constant Speed Baseline');
plot(socConst, '-g', 'DisplayName', 'Constant SoC Baseline');
plot(mpc, '-b', 'DisplayName', 'mpc v1');
plot(maxSpeed, '-k', 'DisplayName', 'Theoretical Best');
plot(mpc_v2, 'DisplayName', 'mpc v2');
title('Distance Traveled vs Time');
xlabel('Time Minutes');
ylabel('Distance Meters');
legend('Location', 'northwest');