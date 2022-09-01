clear;clc; close all;


% Open 2750Wh Results
fig(1) = openfig("1_5_N_Uniform_Results\autoSpeedSelect_3500.fig");
fig(2) = openfig("1_5_E_Uniform_Results\autoSpeedSelect_3500.fig");
fig(3) = openfig("1_5_N_1_5_E_Uniform_Results\autoSpeedSelect_3500.fig");

new_fig = figure;
ax_new = gobjects(size(fig));
for i = 1:3
    ax = subplot(1,3,i);
    ax_old = findobj(fig(i), 'type', 'axes');
    ax_new(i) = copyobj(ax_old, new_fig);
    ax_new(i).YLimMode = 'manual';
    ax_new(i).Position = ax.Position;
    ax_new(i).Position(4) = ax_new(i).Position(4)-0.02;
    delete(ax);
end