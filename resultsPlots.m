clear; clc; close all;

%% Load data
load('2022-09-14_12-30 (4.5kts).mat');
const4_5 = distCum;
clearvars -except const4_5

load('2022-09-14_19-39 (soc_constant).mat');
socConst = distCum;
clearvars -except const4_5 socConst

load('2022-09-16_07-33 (MPC_Test1).mat');
mpc = distCum;
clearvars -except const4_5 socConst mpc

%% Plot Data
hold on;
plot(const4_5, '-r', 'DisplayName', 'const4_5');
plot(socConst, '-g', 'DisplayName', 'socConst');
plot(mpc, '-b', 'DisplayName', 'mpc');
title('Distance Traveled vs Time');
xlabel('Time Minutes');
ylabel('Distance Meters');