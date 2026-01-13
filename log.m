% =========================================================
% Plot TRUE vs ESTIMATE states from CSV + error plots
% Expected columns (case-insensitive):
% t n e d vn ve vd phi theta psi p q r phiE thetaE psiE nE eE dE vnE veE vdE
% =========================================================
clear; clc; close all;

% -------- File picker --------
[file, path] = uigetfile({'*.csv','CSV files (*.csv)'}, 'Select simulation CSV file');
if isequal(file,0)
    error('No file selected');
end
filename = fullfile(path,file);

% -------- Load CSV robustly --------
opts = detectImportOptions(filename, 'NumHeaderLines', 0);
opts.VariableNamingRule = 'preserve';   % keep header text as-is if possible
Traw = readtable(filename, opts);

% Also make a version with MATLAB-valid variable names for flexible access
T = Traw;
T.Properties.VariableNames = matlab.lang.makeValidName(T.Properties.VariableNames);

% -------- Helper: robust column getter (handles preserve/makeValidName/case) --------
getcol = @(Tbl, name) local_getcol(Tbl, name);

% -------- Time --------
t = getcol(T, 't');
if all(isnan(t))
    % fallback: assume first column is time
    t = T{:,1};
end

% -------- True states --------
n     = getcol(T,'n');     e     = getcol(T,'e');     d     = getcol(T,'d');
vn    = getcol(T,'vn');    ve    = getcol(T,'ve');    vd    = getcol(T,'vd');
phi   = getcol(T,'phi');   theta = getcol(T,'theta'); psi   = getcol(T,'psi');
p     = getcol(T,'p');     q     = getcol(T,'q');     r     = getcol(T,'r');

% -------- Estimated states (if present) --------
nE    = getcol(T,'nE');    eE    = getcol(T,'eE');    dE    = getcol(T,'dE');
vnE   = getcol(T,'vnE');   veE   = getcol(T,'veE');   vdE   = getcol(T,'vdE');
phiE  = getcol(T,'phiE');  thetaE= getcol(T,'thetaE');psiE  = getcol(T,'psiE');

% -------- Thrust Commands --------
T1 = getcol(T,'T1');
T2 = getcol(T,'T2');
T3 = getcol(T,'T3');
T4 = getcol(T,'T4');

% -------- Optional: unwrap yaw for nicer overlays --------
psi_u  = unwrap(psi);
psiE_u = unwrap(psiE);

% -------- Errors (only where both exist) --------
en   = nE - n;
ee   = eE - e;
ed   = dE - d;
evn  = vnE - vn;
eve  = veE - ve;
evd  = vdE - vd;

% For angles, compute wrapped difference in [-pi, pi]
ephi   = local_wrapToPi(phiE - phi);
etheta = local_wrapToPi(thetaE - theta);
epsi   = local_wrapToPi(psiE_u - psi_u);

% =========================================================
% PLOTS: True vs Estimated (overlay) + Errors
% =========================================================

% -------- Position --------
local_plot_overlay_and_error(t, ...
    {'n','e','d'}, {n,e,d}, {nE,eE,dE}, ...
    {'nE-n','eE-e','dE-d'}, {en,ee,ed}, ...
    'Position (NED)');

% -------- Velocity --------
local_plot_overlay_and_error(t, ...
    {'vn','ve','vd'}, {vn,ve,vd}, {vnE,veE,vdE}, ...
    {'vnE-vn','veE-ve','vdE-vd'}, {evn,eve,evd}, ...
    'Velocity (NED)');

% -------- Attitude (Euler) --------
% Use unwrapped yaw for overlay, wrapped error for yaw
local_plot_overlay_and_error(t, ...
    {'phi','theta','psi'}, {phi,theta,psi_u}, {phiE,thetaE,psiE_u}, ...
    {'phiE-phi','thetaE-theta','wrap(psiE-psi)'}, {ephi,etheta,epsi}, ...
    'Attitude (Euler angles)');

% -------- Body rates (usually no estimated columns in your header) --------
% Plot true only; if you later log pE/qE/rE, it will overlay automatically if present.
pE = getcol(T,'pE'); qE = getcol(T,'qE'); rE = getcol(T,'rE');
ep = pE - p; eq = qE - q; er = rE - r;

local_plot_overlay_and_error(t, ...
    {'p','q','r'}, {p,q,r}, {pE,qE,rE}, ...
    {'pE-p','qE-q','rE-r'}, {ep,eq,er}, ...
    'Body rates (p,q,r)');

figure('Name','Motor Thrusts T1–T4','Color','w'); hold on; grid on;
plot(t, T1, 'LineWidth', 1.25);
plot(t, T2, 'LineWidth', 1.25);
plot(t, T3, 'LineWidth', 1.25);
plot(t, T4, 'LineWidth', 1.25);

xlabel('t [s]');
ylabel('Thrust');
title('Motor Thrust Commands');
legend({'T1','T2','T3','T4'}, 'Location','best');
% =========================================================
% 3D POSITION PLOT COLORED BY TIME
% =========================================================
% Use -d so "Up" is positive in the plot (optional but usually clearer)
z_true = -d;
z_est  = -dE;

figure('Name','3D Position (colored by time)', 'Color','w');
ax = axes; hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'North [m]'); ylabel(ax,'East [m]'); zlabel(ax,'Up [m]');
title(ax,'3D Position (color = time)');

% --- True trajectory colored by time ---
scatter3(ax, n, e, z_true, 18, t, 'filled');   % size=18, color=t
cb = colorbar(ax);
cb.Label.String = 't [s]';

% Optional: thin line to help visualize continuity
plot3(ax, n, e, z_true, 'LineWidth', 0.75);

% --- Estimated trajectory (if present), colored by time too ---
if ~(all(isnan(nE)) || all(isnan(eE)) || all(isnan(dE)))
    scatter3(ax, nE, eE, z_est, 18, t, 'o');   % same colormap via t
    plot3(ax, nE, eE, z_est, '--', 'LineWidth', 0.75);
    legend(ax, {'True (time-colored)','True (line)','Est (time-colored)','Est (line)'}, ...
        'Location','best');
else
    legend(ax, {'True (time-colored)','True (line)'}, 'Location','best');
end

view(ax, 3);

% =========================================================
% Local functions
% =========================================================
function x = local_getcol(T, name)
    % Returns column as double vector if present, else NaNs.
    % Handles exact name, valid-name version, and case-insensitive match.
    if ~istable(T)
        error('local_getcol: input must be a table');
    end

    % Candidate keys to try
    cand = {};
    cand{end+1} = name;                              % as given (e.g., 'phiE')
    cand{end+1} = matlab.lang.makeValidName(name);   % valid-name version
    % Also try lower/upper variants
    cand{end+1} = lower(cand{end});
    cand{end+1} = upper(cand{end});

    vars = T.Properties.VariableNames;

    % 1) direct matches
    for k = 1:numel(cand)
        idx = find(strcmp(vars, cand{k}), 1);
        if ~isempty(idx)
            x = T.(vars{idx});
            x = double(x);
            return;
        end
    end

    % 2) case-insensitive match
    for k = 1:numel(cand)
        idx = find(strcmpi(vars, cand{k}), 1);
        if ~isempty(idx)
            x = T.(vars{idx});
            x = double(x);
            return;
        end
    end

    % Not found
    x = nan(height(T),1);
end

function y = local_wrapToPi(x)
    % Wrap angle to [-pi, pi]
    y = mod(x + pi, 2*pi) - pi;
end

function local_plot_overlay_and_error(t, labels, y_true, y_est, err_labels, y_err, figTitle)
    % Overlays True vs Est when est is available; always plots errors when possible.
    % y_true/y_est/y_err are 1x3 cell arrays of vectors.

    figure('Name', figTitle, 'Color', 'w');
    tiledlayout(2,3, 'Padding','compact', 'TileSpacing','compact');

    % --- Overlay row ---
    for i = 1:3
        nexttile(i);
        plot(t, y_true{i}, 'LineWidth', 1.25); hold on;
        if ~(all(isnan(y_est{i})) || isempty(y_est{i}))
            plot(t, y_est{i}, '--', 'LineWidth', 1.25);
            legend('True','Est','Location','best');
        else
            legend('True','Location','best');
        end
        grid on;
        xlabel('t [s]');
        ylabel(labels{i});
        title([labels{i} ' (True vs Est)']);
    end

    % --- Error row ---
    for i = 1:3
        nexttile(3+i);
        if all(isnan(y_err{i})) || isempty(y_err{i})
            plot(t, nan(size(t))); % placeholder
            title([err_labels{i} ' (missing)']);
        else
            plot(t, y_err{i}, 'LineWidth', 1.25);
            title([err_labels{i} ' (Error)']);
        end
        grid on;
        xlabel('t [s]');
        ylabel('Error');
    end
end
