clc; clear; close all
% udp_live_plot_3dpos.m

port = 8080;
u = udpport("datagram","IPV4","LocalPort",port);
disp("Listening on UDP port " + port);

plotEvery = 1;

% Ring buffer length for trail
N = 25;
pe_n  = nan(1,N);
pe_e  = nan(1,N);
pe_d  = nan(1,N);
k = 0;

% =========================
% LOGGING (for post-run plots)
% =========================
t_log   = [];          % [s]
pos_log = [];          % Nx3 [n e d]
vel_log = [];          % Nx3 [n e d]
eul_log = [];          % Nx3 [r p y]
hz_log  = [];          % [Hz]
health_log = [];

% Optional: stop if telemetry disappears for too long
telemetryTimeout_s = 10.0;     % set [] or inf to disable
lastRx_tic = tic;

% =========================
% FIGURE + LAYOUT
% =========================
figure(1); clf;
fig = gcf;
fig.Color = [0.08 0.08 0.10];

% 3D AXES
ax = axes('Parent',fig);
hold(ax,"on"); grid(ax,"on"); view(ax,3);

ax.Color = [0.10 0.10 0.12];
ax.XColor = [0.9 0.9 0.9];
ax.YColor = [0.9 0.9 0.9];
ax.ZColor = [0.9 0.9 0.9];
ax.FontName = 'Consolas';

xlabel(ax,"E"); ylabel(ax,"N"); zlabel(ax,"-D");
title(ax,"Live Position",'Color',[1 1 1]);

axis(ax,"auto");
xlim(ax,[-1 6]); ylim(ax,[-1 11]); zlim(ax,[0 5]);
daspect(ax,[1 1 1]);
rotate3d(ax,"on");

% Trail
hTrail = plot3(ax,nan,nan,nan,'-','LineWidth',1.5,'Color',[0.9 0.85 0.98]);

% Ground projection marker (shadow on ground plane Z=0 in plot frame)
hGround = plot3(ax, nan, nan, nan, 'o', ...
    'MarkerSize', 10, ...
    'MarkerFaceColor', [0.95 0.75 0.15], ...
    'MarkerEdgeColor', [0.10 0.10 0.10], ...
    'LineWidth', 1.0);

% =========================
% Quadcopter "X" + Body Axes (RGB)
% =========================
armSpan  = 0.5;   % total tip-to-tip span of the drawn X (meters in your plot units)
axisLen  = 0.25;  % length of body axes triad (meters in your plot units)
armLW    = 2.0;

% Two crossing arms (diagonals in body frame -> looks like an "X" top-down)
hArm1 = plot3(ax, nan, nan, nan, '-', 'LineWidth', armLW, 'Color', [0.85 0.85 0.90]);
hArm2 = plot3(ax, nan, nan, nan, '-', 'LineWidth', armLW, 'Color', [0.85 0.85 0.90]);

% Body axes: x=forward (red), y=right (green), z=down (blue)
hBx = plot3(ax, nan, nan, nan, '-', 'LineWidth', 2.2, 'Color', [1 0 0]);
hBy = plot3(ax, nan, nan, nan, '-', 'LineWidth', 2.2, 'Color', [0 1 0]);
hBz = plot3(ax, nan, nan, nan, '-', 'LineWidth', 2.2, 'Color', [0 0 1]);

% Telemetry text
hText = annotation('textbox',[0.015 0.70 0.30 0.28],...
    'String','Waiting for data...','FontName','Consolas',...
    'EdgeColor',[0.3 0.3 0.3],'BackgroundColor',[0.12 0.12 0.14],...
    'Color',[0.95 0.95 0.95]);

% MissionPhase enum mapping (0-based enum → +1 for MATLAB)
phaseNames = {
    'Takeoff'
    'Sweep'
    'GoToTarget'
    'HoverAtTarget'
    'DescendToTarget'
    'Drop'
    'GoToLand'
    'DescendToLand'
    'Terminate'
};

% NavMode enum mapping
modeNames = {
    'Manual'
    'Waypoint'
    'Sweep'
};

armNames = {'DISARMED'
          'ARMED'};

% =========================
% Dotted volume box (3D)
% =========================
Emax = 4.572;
Nmax = 9.144;
Zmin = 0;
Zmax = 5;

corners = [ ...
    0    0    Zmin;   % 1
    Emax 0    Zmin;   % 2
    Emax Nmax Zmin;   % 3
    0    Nmax Zmin;   % 4
    0    0    Zmax;   % 5
    Emax 0    Zmax;   % 6
    Emax Nmax Zmax;   % 7
    0    Nmax Zmax];  % 8

edges = [ ...
    1 2; 2 3; 3 4; 4 1; ...   % bottom rectangle
    5 6; 6 7; 7 8; 8 5; ...   % top rectangle
    1 5; 2 6; 3 7; 4 8];      % verticals

for i = 1:size(edges,1)
    p1 = corners(edges(i,1),:);
    p2 = corners(edges(i,2),:);
    plot3(ax, [p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
        ':', 'LineWidth', 1.2, 'Color', [0.3 0.3 0.3]);
end

% =========================
% Helper: body->NED rotation (3-2-1 yaw-pitch-roll)
% =========================
% ---- NED -> Plot mapping (Plot axes are [E, N, Up] = [E, N, -D]) ----
T_plot_ned = [ 0 1 0;   % E  = e
               1 0 0;   % N  = n
               0 0 -1]; % Up = -d

Rx = @(phi) [1 0 0;
             0 cos(phi) sin(phi);
             0 -sin(phi) cos(phi)];

Ry = @(th)  [ cos(th) 0 -sin(th);
              0      1  0;
              sin(th) 0  cos(th)];

Rz = @(psi) [ cos(psi)  sin(psi) 0;
             -sin(psi)  cos(psi) 0;
              0         0        1];

% If your angles are EXTRINSIC about inertial axes (common telemetry behavior):
rotmBodyToNED = @(phi,th,psi) Rz(psi) * Ry(th) * Rx(phi);

% =========================
% MAIN LOOP
% =========================
while ishandle(ax)
    if u.NumDatagramsAvailable == 0
        % Optional timeout exit if telemetry stops
        if ~isempty(telemetryTimeout_s) && isfinite(telemetryTimeout_s)
            if toc(lastRx_tic) > telemetryTimeout_s
                disp("Telemetry timeout. Exiting main loop.");
                break;
            end
        end
        pause(0.01); continue;
    end

    d = read(u,1,"uint8");
    disp(class(d.Data));
    disp(size(d.Data));
    disp(numel(d.Data));

    try
        s = jsondecode(native2unicode(d.Data(:).',"UTF-8"));

        % mark telemetry alive
        lastRx_tic = tic;

        % =========================
        % LOG THIS SAMPLE
        % =========================
        t_log(end+1,1)      = s.t;
        pos_log(end+1,1:3)  = reshape(s.pos_est,1,3);
        vel_log(end+1,1:3)  = reshape(s.vel_est,1,3);
        eul_log(end+1,1:3)  = reshape(s.euler_est,1,3);
        hz_log(end+1,1)     = s.Hz;
        health_log(end+1,1) = s.EKF_Health;

        k = k + 1;
        idx = mod(k-1,N)+1;

        pe_n(idx) = s.pos_est(1);
        pe_e(idx) = s.pos_est(2);
        pe_d(idx) = s.pos_est(3);

        if mod(k,plotEvery)~=0
            continue;
        end

        if k < N
            ii = 1:k;
        else
            ii = [idx+1:N 1:idx];
        end

        % Plot coordinates are [E, N, -D]
        X = pe_e(ii); Y = pe_n(ii); Z = -pe_d(ii);
        set(hTrail,'XData',X,'YData',Y,'ZData',Z);

        phi = s.euler_est(1);
        th  = s.euler_est(2);
        psi = s.euler_est(3);

        p_ned = [s.pos_est(1); s.pos_est(2); s.pos_est(3)];
        p_plot = [p_ned(2); p_ned(1); -p_ned(3)];
        set(hGround, 'XData', p_plot(1), 'YData', p_plot(2), 'ZData', 0);

        Rnb     = rotmBodyToNED(-phi, -th, -psi);   % body -> NED
        Rplot_b = T_plot_ned * Rnb;                 % body -> plot

        p_plot = [s.pos_est(2); s.pos_est(1); -s.pos_est(3)];

        a = armSpan/2;
        d1_b = (a/sqrt(2))*[ 1;  1; 0];
        d2_b = (a/sqrt(2))*[ 1; -1; 0];

        p1a = p_plot - Rplot_b*d1_b;  p1b = p_plot + Rplot_b*d1_b;
        p2a = p_plot - Rplot_b*d2_b;  p2b = p_plot + Rplot_b*d2_b;

        set(hArm1,'XData',[p1a(1) p1b(1)],'YData',[p1a(2) p1b(2)],'ZData',[p1a(3) p1b(3)]);
        set(hArm2,'XData',[p2a(1) p2b(1)],'YData',[p2a(2) p2b(2)],'ZData',[p2a(3) p2b(3)]);

        ex_b = [1;0;0]; ey_b = [0;1;0]; ez_b = [0;0;1];

        px = p_plot + axisLen*(Rplot_b*ex_b);
        py = p_plot + axisLen*(Rplot_b*ey_b);
        pz = p_plot + axisLen*(Rplot_b*ez_b);

        set(hBx,'XData',[p_plot(1) px(1)],'YData',[p_plot(2) px(2)],'ZData',[p_plot(3) px(3)]);
        set(hBy,'XData',[p_plot(1) py(1)],'YData',[p_plot(2) py(2)],'ZData',[p_plot(3) py(3)]);
        set(hBz,'XData',[p_plot(1) pz(1)],'YData',[p_plot(2) pz(2)],'ZData',[p_plot(3) pz(3)]);

        phaseStr = phaseNames{s.phase + 1};
        modeStr  = modeNames{s.mode  + 1};
        armStr   = armNames{s.armed + 1};

        txt = sprintf([ ...
            't [s]: %.2f ','Rate [Hz]: %.2f\n' ...
            '%s | phase: %s | mode: %s\n','EKF Health: %.2f \n' ...
            'pos [n e d]: [% .2f % .2f % .2f]\n' ...
            'vel [n e d]: [% .2f % .2f % .2f]\n' ...
            'euler [r p y]: [% .2f % .2f % .2f]'], ...
            s.t, ...
            s.Hz, ...
            armStr, ...
            phaseStr, ...
            modeStr, ...
            s.EKF_Health,...
            s.pos_est(1), s.pos_est(2), s.pos_est(3), ...
            s.vel_est(1), s.vel_est(2), s.vel_est(3), ...
            s.euler_est(1), s.euler_est(2), s.euler_est(3));

        set(hText,'String',txt);

        drawnow limitrate

    catch ME
        disp(ME.message);
    end
end
%%
% =========================
% POST-RUN PLOTS (after loop exits)
% =========================
if ~isempty(t_log)
    [t_log, order] = sort(t_log);
    pos_log = pos_log(order,:);
    vel_log = vel_log(order,:);
    eul_log = eul_log(order,:);
    hz_log  = hz_log(order,:);
    health_log = health_log(order,:);

    % Position
    figure('Name','Telemetry Log - Position'); clf;
    plot(t_log, pos_log(:,1), t_log, pos_log(:,2), t_log, pos_log(:,3), 'LineWidth', 1.2);
    grid on; xlabel('t [s]'); ylabel('pos [m]'); legend('n','e','d','Location','best');
    title('Position vs Time');

    % Velocity
    figure('Name','Telemetry Log - Velocity'); clf;
    plot(t_log, vel_log(:,1), t_log, vel_log(:,2), t_log, vel_log(:,3), 'LineWidth', 1.2);
    grid on; xlabel('t [s]'); ylabel('vel [m/s]'); legend('n','e','d','Location','best');
    title('Velocity vs Time');

    % Euler
    figure('Name','Telemetry Log - Euler'); clf;
    plot(t_log, eul_log(:,1), t_log, eul_log(:,2), t_log, eul_log(:,3), 'LineWidth', 1.2);
    grid on; xlabel('t [s]'); ylabel('euler [rad]'); legend('roll','pitch','yaw','Location','best');
    title('Euler Angles vs Time');

    % Hz
    figure('Name','Telemetry Log - Rate'); clf;
    plot(t_log, hz_log, 'LineWidth', 1.2);
    grid on; xlabel('t [s]'); ylabel('Rate [Hz]');
    title('Telemetry Rate vs Time');

    % Health (NIS) vs time
    figure('Name','EKF Health Log'); clf;
    t_health = t_log(1:numel(health_log));
    plot(t_health, health_log, 'LineWidth', 1.2);
    hold on;
    meanHealth = mean(health_log,'omitnan');
    plot(t_health, meanHealth*ones(size(t_health)), 'LineWidth', 2);
    grid on;
    xlabel('t [s]');
    ylabel('NIS');
    title('EKF Health vs Time');
    legend('NIS','Mean NIS');

    % ============================================================
    % NEW: Actual vs Ideal distribution comparison (m = 4)
    %   - Histogram (PDF) overlay with chi-square(4) PDF
    %   - Empirical CDF vs chi-square(4) CDF
    %   - Q-Q plot vs chi-square(4)
    % ============================================================
    m = 4;

    nis = health_log(:);
    nis = nis(isfinite(nis));

    % (Optional) drop initial transient (set to 0 to disable)
    transient_s = 10.0;
    if transient_s > 0 && ~isempty(t_health)
        keep = t_health(:) >= (t_health(1) + transient_s);
        nis = health_log(keep);
        nis = nis(:);
        nis = nis(isfinite(nis));
    end

    if numel(nis) >= 10
        % ---- Figure: PDF (hist) vs chi^2 PDF ----
        figure('Name','NIS Distribution vs Chi-square(4) (PDF)'); clf;
        grid on; hold on;

        % Histogram normalized to PDF
        histogram(nis, 'Normalization', 'pdf');

        % Theoretical chi-square PDF over a sensible range
        x_max = max([quantile(nis,0.995) chi2inv(0.995,m)]);
        x = linspace(0, max(1e-6, x_max), 400);
        plot(x, chi2pdf(x,m), 'LineWidth', 2);

        xlabel('NIS');
        ylabel('PDF');
        title('Actual NIS PDF vs Ideal \chi^2(4) PDF');
        legend('Actual (hist PDF)','Ideal \chi^2(4) PDF','Location','best');

        % ---- Figure: CDF comparison ----
        figure('Name','NIS Distribution vs Chi-square(4) (CDF)'); clf;
        grid on; hold on;

        [Femp, Xemp] = ecdf(nis);
        plot(Xemp, Femp, 'LineWidth', 2);
        plot(Xemp, chi2cdf(Xemp,m), '--', 'LineWidth', 2);

        xlabel('NIS');
        ylabel('CDF');
        title('Empirical CDF vs Ideal \chi^2(4) CDF');
        legend('Empirical CDF','Ideal \chi^2(4) CDF','Location','best');

        % ---- Figure: Q-Q plot ----
        figure('Name','NIS Q-Q vs Chi-square(4)'); clf;
        grid on; hold on;

        nis_s = sort(nis);
        n = numel(nis_s);
        p = ((1:n)' - 0.5) / n;
        q = chi2inv(p, m);

        plot(q, nis_s, '.', 'MarkerSize', 10);
        lo = min([q; nis_s]); hi = max([q; nis_s]);
        plot([lo hi],[lo hi],'--','LineWidth',2);

        xlabel('Theoretical \chi^2(4) quantiles');
        ylabel('Empirical NIS quantiles');
        title('Q-Q: Actual NIS vs Ideal \chi^2(4)');
        legend('Samples','y = x','Location','best');

        % ---- Quick numeric checks (optional) ----
        chi2_95 = chi2inv(0.95,m);
        chi2_99 = chi2inv(0.99,m);

        fprintf('\n=== NIS Consistency Check (m=%d) ===\n', m);
        fprintf('N = %d samples\n', numel(nis));
        fprintf('Mean NIS: %.3f (ideal %.1f)\n', mean(nis,'omitnan'), m);
        fprintf('P(NIS > 95%%): %.3f (ideal 0.050)  threshold=%.3f\n', mean(nis > chi2_95), chi2_95);
        fprintf('P(NIS > 99%%): %.3f (ideal 0.010)  threshold=%.3f\n', mean(nis > chi2_99), chi2_99);
        fprintf('Median NIS: %.3f (ideal %.3f)\n', median(nis,'omitnan'), chi2inv(0.5,m));
    else
        disp("Not enough NIS samples to make distribution plots.");
    end

else
    disp("No telemetry logged; nothing to plot.");
end