clear; clc; close all;

%% Load data
load('2022-09-16_09-26 (4.5kts).mat');
const4_5 = distCum;
clearvars -except const4_5

load('2022-09-16_09-38 (soc_constant).mat');
socConst = distCum;
clearvars -except const4_5 socConst

load('2022-09-16_07-33 (MPC_Test1).mat');
mpc = distCum;
clearvars -except const4_5 socConst mpc

load('2022-09-19_13-30 (4.5 const, no battery constraint).mat');
maxSpeed = distCum;
clearvars -except const4_5 socConst mpc maxSpeed

load('2022-09-20_06-00 (unconstrained basline).mat');
baseline = distCum;
clearvars -except const4_5 socConst mpc maxSpeed baseline

%% Plot Data
hold on;
plot(const4_5, '-r', 'DisplayName', 'const4_5');
plot(socConst, '-g', 'DisplayName', 'socConst');
plot(mpc, '-b', 'DisplayName', 'mpc');
plot(maxSpeed, '-k', 'DisplayName', 'maxSpeed');
plot(baseline, 'DisplayName', 'baseline');
title('Distance Traveled vs Time');
xlabel('Time Minutes');
ylabel('Distance Meters');