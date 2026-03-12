% =========================================================
% GAIT CYCLE PLOTS
% MCG4366 Group 12
%
% Generates six figures for the prosthetic knee design:
%   1.  Joint Reaction Force (Fx, Fy, |F|) during gait
%   2.  Knee joint moment during gait
%   3.  Knee joint angle during gait
%   4.  Hydraulic pressure + damping torque (variable stiffness)
%   5.  Range of motion throughout the gait cycle
%   6.  Servo motor needle force during gait
%
% All plots share the same x-axis: gait cycle percentage (0-100%).
% Background shading distinguishes stance (0-60%) from swing (60-100%).
%
% Usage (called automatically from Main.getResults):
%   gaitSeries    = JointReactionForce.getGaitTimeSeries(BW, H);
%   pressureSeries = HydraulicAnalysis.getPressureTimeSeries(d_eff);
%   GaitPlots.plotAll(gaitSeries, pressureSeries);
% =========================================================

classdef GaitPlots

methods (Static)

% =========================================================
%  PLOT ALL — convenience wrapper that generates all 6 figures
% =========================================================
function plotAll(gaitSeries, pressureSeries)
    GaitPlots.plotJRF(gaitSeries);
    GaitPlots.plotJointMoment(gaitSeries);
    GaitPlots.plotJointAngle(gaitSeries);
    GaitPlots.plotHydraulicPressure(pressureSeries);
    GaitPlots.plotRangeOfMotion(gaitSeries);
    GaitPlots.plotServoForce(pressureSeries);
end

% =========================================================
%  PLOT ALL INTO AXES — draws all 6 plots into provided uiaxes FOR GUI
%  axArray(1)=JRF, (2)=Moment, (3)=Angle,
%  axArray(4)=Hydraulic, (5)=ROM, (6)=Servo
% =========================================================
function plotAllInAxes(gaitSeries, pressureSeries, axArray)
    GaitPlots.drawJRF(gaitSeries, axArray(1));
    GaitPlots.drawJointMoment(gaitSeries, axArray(2));
    GaitPlots.drawJointAngle(gaitSeries, axArray(3));
    GaitPlots.drawHydraulicPressure(pressureSeries, axArray(4));
    GaitPlots.drawRangeOfMotion(gaitSeries, axArray(5));
    GaitPlots.drawServoForce(pressureSeries, axArray(6));
end

function drawJRF(series, ax)
    cla(ax);
    hold(ax, 'on');
    plot(ax, series.gait_pct, series.Fx_k,      'b-',  'LineWidth', 1.5, 'DisplayName', 'F_x  (horiz.)');
    plot(ax, series.gait_pct, series.Fy_k,      'r-',  'LineWidth', 1.5, 'DisplayName', 'F_y  (vert.)');
    plot(ax, series.gait_pct, series.F_k_total, 'k--', 'LineWidth', 2,   'DisplayName', '|F|  (resultant)');
    yline(ax, 0, 'Color', [0.6 0.6 0.6], 'LineStyle', ':', 'HandleVisibility', 'off');
    hold(ax, 'off');
    ax.XLabel.String = 'Gait Cycle (%)';
    ax.YLabel.String = 'Force (N)';
    ax.Title.String  = 'Knee Joint Reaction Force During Gait';
    legend(ax, 'Location', 'best');
    ax.XGrid = 'on'; ax.YGrid = 'on';
    ax.XLim  = [0 100];
    drawnow;
    GaitPlots.shadingOnAx(ax);
end

function drawJointMoment(series, ax)
    cla(ax);
    hold(ax, 'on');
    plot(ax, series.gait_pct, series.M_k, 'b-', 'LineWidth', 1.5, 'DisplayName', 'M_k');
    yline(ax, 0, 'Color', [0.6 0.6 0.6], 'LineStyle', ':', 'HandleVisibility', 'off');
    [~, idx] = max(abs(series.M_k));
    plot(ax, series.gait_pct(idx), series.M_k(idx), 'ro', 'MarkerSize', 8, 'LineWidth', 2, ...
         'DisplayName', sprintf('Peak: %.1f N·m (%.0f%%)', series.M_k(idx), series.gait_pct(idx)));
    hold(ax, 'off');
    ax.XLabel.String = 'Gait Cycle (%)';
    ax.YLabel.String = 'Knee Moment (N·m)';
    ax.Title.String  = 'Knee Joint Moment During Gait';
    legend(ax, 'Location', 'best');
    ax.XGrid = 'on'; ax.YGrid = 'on';
    ax.XLim  = [0 100];
    drawnow;
    GaitPlots.shadingOnAx(ax);
end

function drawJointAngle(series, ax)
    cla(ax);
    hold(ax, 'on');
    plot(ax, series.gait_pct, series.theta_joint, 'b-', 'LineWidth', 1.5, 'DisplayName', '\theta_{knee}');
    hold(ax, 'off');
    ax.XLabel.String = 'Gait Cycle (%)';
    ax.YLabel.String = 'Knee Joint Angle (°)';
    ax.Title.String  = 'Knee Joint Angle During Gait';
    legend(ax, 'Location', 'best');
    ax.XGrid = 'on'; ax.YGrid = 'on';
    ax.XLim  = [0 100];
    drawnow;
    GaitPlots.shadingOnAx(ax);
end

function drawHydraulicPressure(pressureSeries, ax)
    cla(ax);
    yyaxis(ax, 'left');
    hold(ax, 'on');
    plot(ax, pressureSeries.gait_pct, pressureSeries.P_series / 1e6, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Pressure [MPa]');
    ax.YLabel.String = 'Hydraulic Pressure (MPa)';
    yyaxis(ax, 'right');
    plot(ax, pressureSeries.gait_pct, pressureSeries.T_damp_series, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Damping torque [N·m]');
    ax.YLabel.String = 'Damping Torque (N·m)';
    yyaxis(ax, 'left');
    hold(ax, 'off');
    ax.XLabel.String = 'Gait Cycle (%)';
    ax.Title.String  = 'Hydraulic Pressure & Damping Torque (Variable Stiffness)';
    legend(ax, 'Location', 'best');
    ax.XGrid = 'on'; ax.YGrid = 'on';
    ax.XLim  = [0 100];
    drawnow;
    GaitPlots.shadingOnAx(ax);
end

function drawRangeOfMotion(series, ax)
    theta_min = min(series.theta_joint);
    theta_max = max(series.theta_joint);
    ROM       = theta_max - theta_min;
    cla(ax);
    hold(ax, 'on');
    x_fill = [series.gait_pct; flipud(series.gait_pct)];
    y_fill = [repmat(theta_min, size(series.gait_pct)); flipud(series.theta_joint)];
    patch(ax, x_fill, y_fill, [0.7 0.85 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.6, ...
          'DisplayName', 'Flexion envelope');
    plot(ax, series.gait_pct, series.theta_joint, 'b-', 'LineWidth', 2, 'DisplayName', '\theta_{knee}');
    yline(ax, theta_min, 'k--', 'LineWidth', 1, 'DisplayName', sprintf('Min θ = %.1f°', theta_min));
    yline(ax, theta_max, 'k--', 'LineWidth', 1, 'DisplayName', sprintf('Max θ = %.1f°', theta_max));
    hold(ax, 'off');
    ax.XLabel.String = 'Gait Cycle (%)';
    ax.YLabel.String = 'Knee Joint Angle (°)';
    ax.Title.String  = sprintf('Range of Motion — ROM = %.1f°', ROM);
    legend(ax, 'Location', 'best');
    ax.XGrid = 'on'; ax.YGrid = 'on';
    ax.XLim  = [0 100];
    drawnow;
    GaitPlots.shadingOnAx(ax);
end

function drawServoForce(pressureSeries, ax)
    cla(ax);
    hold(ax, 'on');
    plot(ax, pressureSeries.gait_pct, pressureSeries.F_needle_series, 'm-', 'LineWidth', 1.5, ...
         'DisplayName', 'Needle force F_{axial}');
    [F_max, idx] = max(pressureSeries.F_needle_series);
    plot(ax, pressureSeries.gait_pct(idx), F_max, 'ro', 'MarkerSize', 8, 'LineWidth', 2, ...
         'DisplayName', sprintf('Peak: %.4f N at %.0f%%', F_max, pressureSeries.gait_pct(idx)));
    hold(ax, 'off');
    ax.XLabel.String = 'Gait Cycle (%)';
    ax.YLabel.String = 'Axial Needle Force (N)';
    ax.Title.String  = 'Servo Motor Needle Force During Gait';
    legend(ax, 'Location', 'best');
    ax.XGrid = 'on'; ax.YGrid = 'on';
    ax.XLim  = [0 100];
    drawnow;
    GaitPlots.shadingOnAx(ax);
end

% Shared phase shading — works on any axes handle (regular or uiaxes)
function shadingOnAx(ax)
    y  = ax.YLim;
    h1 = patch(ax, [0 60 60 0],     [y(1) y(1) y(2) y(2)], [0.98 0.96 0.80], ...
               'EdgeColor', 'none', 'FaceAlpha', 0.35, 'HandleVisibility', 'off');
    h2 = patch(ax, [60 100 100 60], [y(1) y(1) y(2) y(2)], [0.80 0.96 0.98], ...
               'EdgeColor', 'none', 'FaceAlpha', 0.35, 'HandleVisibility', 'off');
    h1.ZData = -ones(1,4);
    h2.ZData = -ones(1,4);
    y_label = y(1) + 0.04*(y(2)-y(1));
    text(ax, 30, y_label, 'Stance', 'HorizontalAlignment', 'center', ...
         'Color', [0.5 0.4 0], 'FontSize', 8, 'FontAngle', 'italic');
    text(ax, 80, y_label, 'Swing',  'HorizontalAlignment', 'center', ...
         'Color', [0 0.4 0.5], 'FontSize', 8, 'FontAngle', 'italic');
end

% =========================================================
%  PLOT 1 — JOINT REACTION FORCE DURING GAIT
% =========================================================
% Plots the horizontal (Fx), vertical (Fy), and resultant (|F|)
% knee joint reaction force across the gait cycle.
% The JRF is computed by the inverse-dynamics chain in
% JointReactionForce.getGaitTimeSeries(), propagating Newton's
% second law from the foot segment up to the knee.
function plotJRF(series)
    figure('Name', 'Joint Reaction Force During Gait', 'NumberTitle', 'off');
    hold on;
    plot(series.gait_pct, series.Fx_k,      'b-',  'LineWidth', 1.5, 'DisplayName', 'F_x  (horiz.)');
    plot(series.gait_pct, series.Fy_k,      'r-',  'LineWidth', 1.5, 'DisplayName', 'F_y  (vert.)');
    plot(series.gait_pct, series.F_k_total, 'k--', 'LineWidth', 2,   'DisplayName', '|F|  (resultant)');
    yline(0, 'Color', [0.6 0.6 0.6], 'LineStyle', ':', 'HandleVisibility', 'off');
    GaitPlots.addPhaseShading();
    hold off;
    xlabel('Gait Cycle (%)');
    ylabel('Force (N)');
    title('Knee Joint Reaction Force During Gait');
    legend('Location', 'best');
    grid on;
    xlim([0 100]);
end

% =========================================================
%  PLOT 2 — KNEE JOINT MOMENT DURING GAIT
% =========================================================
% Plots the internal knee moment M_k at every gait frame.
% Positive M_k = extension moment (quadriceps dominant).
% Negative M_k = flexion moment (hamstrings dominant).
% The peak magnitude is the governing design load for the
% shaft, frame, and hydraulic cylinder sizing.
function plotJointMoment(series)
    figure('Name', 'Knee Joint Moment During Gait', 'NumberTitle', 'off');
    hold on;
    plot(series.gait_pct, series.M_k, 'b-', 'LineWidth', 1.5, 'DisplayName', 'M_k');
    yline(0, 'Color', [0.6 0.6 0.6], 'LineStyle', ':', 'HandleVisibility', 'off');
    [~, idx_max] = max(abs(series.M_k));
    plot(series.gait_pct(idx_max), series.M_k(idx_max), 'ro', 'MarkerSize', 8, ...
         'LineWidth', 2, 'DisplayName', sprintf('Peak: %.1f N·m (%.0f%%)', series.M_k(idx_max), series.gait_pct(idx_max)));
    GaitPlots.addPhaseShading();
    hold off;
    xlabel('Gait Cycle (%)');
    ylabel('Knee Moment (N·m)');
    title('Knee Joint Moment During Gait');
    legend('Location', 'best');
    grid on;
    xlim([0 100]);
end

% =========================================================
%  PLOT 3 — KNEE JOINT ANGLE DURING GAIT
% =========================================================
% Plots the knee joint angle (theta_shank - theta_thigh, Winter's
% sign convention) at each gait frame.
% Note: the angles in WinterAppendix_KinKnee.csv are THIGH segment
% absolute angles; the true joint angle requires both segment CSVs.
function plotJointAngle(series)
    figure('Name', 'Knee Joint Angle During Gait', 'NumberTitle', 'off');
    hold on;
    plot(series.gait_pct, series.theta_joint, 'b-', 'LineWidth', 1.5, 'DisplayName', '\theta_{knee}');
    GaitPlots.addPhaseShading();
    hold off;
    xlabel('Gait Cycle (%)');
    ylabel('Knee Joint Angle (°)');
    title('Knee Joint Angle During Gait');
    legend('Location', 'best');
    grid on;
    xlim([0 100]);
end

% =========================================================
%  PLOT 4 — HYDRAULIC PRESSURE AND DAMPING TORQUE
% =========================================================
% Left axis:  hydraulic pressure P(i) [MPa], computed via
%   Hagen-Poiseuille: P = 128·mu·L·Q / (pi·d^4)
%   where Q(i) = A_p · r_ball · |omega_joint(i)|
%
% Right axis: damping torque T(i) = P(i) · A_p · r_ball  [N·m]
%
% The plot demonstrates VARIABLE STIFFNESS: because pressure is
% proportional to angular velocity (via flow rate), the cylinder
% automatically resists faster motion more strongly — passively
% varying the effective knee stiffness throughout the gait cycle.
function plotHydraulicPressure(pressureSeries)
    figure('Name', 'Hydraulic Pressure and Damping Torque During Gait', 'NumberTitle', 'off');

    yyaxis left;
    hold on;
    plot(pressureSeries.gait_pct, pressureSeries.P_series / 1e6, 'b-', 'LineWidth', 1.5);
    ylabel('Hydraulic Pressure (MPa)');

    yyaxis right;
    plot(pressureSeries.gait_pct, pressureSeries.T_damp_series, 'r-', 'LineWidth', 1.5);
    ylabel('Damping Torque (N·m)');

    yyaxis left;  % switch back to left axis so shading uses left-axis ylim
    GaitPlots.addPhaseShading();
    hold off;

    xlabel('Gait Cycle (%)');
    title('Hydraulic Pressure and Damping Torque During Gait (Variable Stiffness)');
    legend({'Pressure [MPa]', 'Damping torque [N·m]'}, 'Location', 'best');
    grid on;
    xlim([0 100]);
end

% =========================================================
%  PLOT 5 — RANGE OF MOTION THROUGHOUT GAIT
% =========================================================
% Shows the knee joint angle trajectory and annotates the total
% range of motion (ROM = theta_max - theta_min).
%
% The shaded region between the maximum-extension baseline and
% the instantaneous angle shows "how much flexion" exists at each
% gait point.  The dashed horizontal lines mark the extremes,
% and the total ROM is displayed in the title.
function plotRangeOfMotion(series)
    theta_min = min(series.theta_joint);
    theta_max = max(series.theta_joint);
    ROM       = theta_max - theta_min;

    figure('Name', 'Range of Motion Throughout Gait', 'NumberTitle', 'off');
    hold on;

    % Shaded fill between minimum angle (baseline) and actual angle
    x_fill = [series.gait_pct; flipud(series.gait_pct)];
    y_fill = [repmat(theta_min, size(series.gait_pct)); flipud(series.theta_joint)];
    patch(x_fill, y_fill, [0.7 0.85 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.6, ...
          'DisplayName', 'Flexion from extension baseline');

    plot(series.gait_pct, series.theta_joint, 'b-', 'LineWidth', 2, ...
         'DisplayName', '\theta_{knee}');
    yline(theta_min, 'k--', 'LineWidth', 1, 'DisplayName', ...
          sprintf('Min θ = %.1f°  (most extended)', theta_min));
    yline(theta_max, 'k--', 'LineWidth', 1, 'DisplayName', ...
          sprintf('Max θ = %.1f°  (peak flexion)', theta_max));

    GaitPlots.addPhaseShading();
    hold off;

    xlabel('Gait Cycle (%)');
    ylabel('Knee Joint Angle (°)');
    title(sprintf('Range of Motion Throughout Gait  —  ROM = %.1f°', ROM));
    legend('Location', 'best');
    grid on;
    xlim([0 100]);
end

% =========================================================
%  PLOT 6 — SERVO MOTOR NEEDLE FORCE DURING GAIT
% =========================================================
% The servo motor controls the axial position of the needle valve.
% At each gait frame the hydraulic pressure P(i) exerts a force on
% the needle face that the servo must hold against:
%
%   F_needle(i) = P(i) × A_seat  = P(i) × pi · d_eff^2 / 4
%
% The peak value determines the minimum stall force specification
% for the servo (e.g. MG90S = 1.8 kgf = 17.6 N).
% This plot shows how servo demand varies with gait phase.
function plotServoForce(pressureSeries)
    figure('Name', 'Servo Motor Needle Force During Gait', 'NumberTitle', 'off');
    hold on;
    plot(pressureSeries.gait_pct, pressureSeries.F_needle_series, 'm-', 'LineWidth', 1.5, ...
         'DisplayName', 'Needle force F_{axial}');
    [F_max, idx_max] = max(pressureSeries.F_needle_series);
    plot(pressureSeries.gait_pct(idx_max), F_max, 'ro', 'MarkerSize', 8, 'LineWidth', 2, ...
         'DisplayName', sprintf('Peak: %.4f N at %.0f%%', F_max, pressureSeries.gait_pct(idx_max)));
    GaitPlots.addPhaseShading();
    hold off;
    xlabel('Gait Cycle (%)');
    ylabel('Axial Needle Force (N)');
    title('Servo Motor Needle Force During Gait');
    legend('Location', 'best');
    grid on;
    xlim([0 100]);
end

% =========================================================
%  HELPER — STANCE / SWING PHASE BACKGROUND SHADING
% =========================================================
% Normal walking gait:  stance = 0-60%,  swing = 60-100%.
% Call AFTER all data plot() calls (inside hold on block) so that
% ylim() is already set by the data, then uistack sends the patches
% to the background.
%
% HandleVisibility is set to 'off' so patches don't appear in the
% legend of the calling plot.
function addPhaseShading()
    y = ylim();
    h1 = patch([0 60 60 0], [y(1) y(1) y(2) y(2)], [0.98 0.96 0.80], ...
               'EdgeColor', 'none', 'FaceAlpha', 0.35, 'HandleVisibility', 'off');
    h2 = patch([60 100 100 60], [y(1) y(1) y(2) y(2)], [0.80 0.96 0.98], ...
               'EdgeColor', 'none', 'FaceAlpha', 0.35, 'HandleVisibility', 'off');
    % ZData = -1 pushes patches behind all lines (Z=0 default).
    % This works with yyaxis dual-axis plots where uistack fails.
    h1.ZData = -ones(1, 4);
    h2.ZData = -ones(1, 4);

    % Phase labels near the bottom of the axes
    y_label = y(1) + 0.04 * (y(2) - y(1));
    text(30, y_label, 'Stance', 'HorizontalAlignment', 'center', ...
         'Color', [0.5 0.4 0], 'FontSize', 8, 'FontAngle', 'italic');
    text(80, y_label, 'Swing', 'HorizontalAlignment', 'center', ...
         'Color', [0 0.4 0.5], 'FontSize', 8, 'FontAngle', 'italic');
end

end
end
