%%% PORSCHE 930 TURBO (3.3L) HYBRID SIMULATION SCRIPT

clear; clc; close all;

%% Load WLTP and Open Model

load("Electric Propulsion Systems - Project\ECE_EUDC_Artemis_Bologna_Roma_WLTC_EE_bus.mat")
open("Porsche930Turbo_Model_V0.slx")

%% BATTERY PACK DATA (High-Power 350V System)
% Cell Type: Molicel P42A 21700 (3.6V nominal, 4.2 Ah, 45A discharge)
nseries   = 96;       % 96 cells in series -> 345.6V Nominal (403.2V Max)
nparallel = 2;        % 96S2P Configuration -> 8.4 Ah (2.90 kWh gross)
V_nom     = nseries * 3.6;          % Nominal Pack Voltage [V] (345.6 V)
Cap_Ah    = nparallel * 4.2;        % Pack Capacity [Ah] (8.4 Ah)
Cap_kWh   = (V_nom * Cap_Ah)/1000;  % Gross Energy Capacity [kWh] (~2.90 kWh)
m_batt    = 26.0;     % Battery pack total weight [kg] (13.4kg cells + 12.6kg packaging)

Ts = 1;

ThermalModelType = 2;        % Activate Box 2 in the Thermal Model inside the battery pack
TRoom            = 25.0;     % Ambient room temperature [°C]
ThermReff        = 25.0;     % Reference temperature for thermal model [°C]

n_cells_tot = 192; % 96S2P total cells

C1Thermal = n_cells_tot * 35.0;  % Core thermal capacitance (~6720 J/K)
C2Thermal = n_cells_tot * 25.0;  % Surface thermal capacitance (~4800 J/K)
R1Therml  = 1.5 / n_cells_tot;   % Internal conduction core-to-surface (~0.0078 °C/W)

% R2 now represents only passive enclosure dissipation to trunk air:
R2Therml  = 0.20;                % Passive enclosure dissipation (~0.2 °C/W)
R3Therml  = 0.002;               % External thermal resistance [°C/W]

%% VEHICLE PARAMETERS (Porsche 930 Turbo 3.3L OEM Specs)
rt = 4.222;                               % Final drive ratio (differential)
gear_ratios = [2.25, 1.30, 0.89, 0.625];  % Gear ratios for the 4-speed Porsche 930
gear_numbers = [1, 2, 3, 4];              
eta_gb = 0.95;                            % Gearbox mechanical efficiency
v_shift_bp   = [0, 18, 33, 48]; % Speed shift thresholds [m/s]
v_shift_bp_EMODE = [0, 5, 22, 35]; 
gear_shift_tbl = [1, 2, 3, 4];   % Gear corresponding [1..4]

% --- Vehicle Masses ---
mc = 1300;            % Curb weight stock Porsche 930 [kg]
mh = 75;              % Standard human mass [kg]
np = 2;               % Number of occupants (Driver + Passenger)
mp_hybrid = 42.5;     % Hybrid hardware (Battery 26.0kg + Motor 12.5kg + Inverter 4.0kg)
mg = mc + (mh * np) + mp_hybrid; % Gross Vehicle Weight [kg] (~1542.5 kg)

% --- Aerodynamics & Chassis ---
ro = 1.204;           % Air density at 20°C [kg/m^3]
cd = 0.39;            % Drag Coefficient (Widebody + Whale Tail Spoiler)
af = 1.77;            % Frontal Area [m^2]
wb = 2.272;           % Wheelbase [m] (Porsche 911 G-Series)
CoG_front = 0.415;    % Front weight distribution (~41.5% with front battery)

% --- Rolling Resistance & Tire Geometry (Rear 245/45 R16 OEM) ---
crr = 0.015;          % Rolling resistance coefficient (Performance tires)
dw  = 16;             % Wheel rim diameter [inches]
wt  = 245;            % Rear tire width [mm]
art = 45;             % Aspect ratio (%)
rw  = (dw * 25.4 / 2 + wt * art / 100) * 1e-3; % Dynamic tire radius [m] (~0.314 m)

%% LOAD FORCE & POWER CALCULATION
grade = 0;            % Road grade (0 = flat road)
alfa  = atan(grade);

dv   = 0.1;           % Speed discretization step [m/s]
v    = 0:dv:75;       % Vehicle speed range [m/s] (0 to 270 km/h)
vkph = v * 3.6;       % Speed in [km/h]

% Force Components
fa     = 0.5 * ro * cd * af * v.^2;                % Aerodynamic Drag Force [N]
frr    = mg * 9.81 * cos(alfa) * crr * ones(size(v)); % Rolling Resistance Force [N]
fgrade = mg * 9.81 * sin(alfa) * ones(size(v));      % Grade Force [N]

fl = fa + frr + fgrade;                           % Total Road Load Force [N]
pl = fl .* v;                                     % Total Road Load Power [W]

pa  = fa .* v;
prr = frr .* v;
pgrade = fgrade .* v;

% Specific Energy Consumption [Wh/km]
cs = (pl ./ vkph) * (1000 / 3600);                % Wh/km (Valid for v > 0)
cs(1) = 0;

%% ICE TORQUE & POWER CURVE (Porsche 930/60 3.3L Flat-6 Turbo)
% OEM Specs: 300 hp (221 kW) @ 5500 RPM | 412 Nm @ 4000 RPM
pmax_ice = 221000;    % Max ICE Power [W] (300 hp)
tmax_ice = 412;       % Max ICE Torque [Nm]

% Turbo lag dynamics and specific fuel consumption
tau_turbo = 0.6;          % Turbocharger time delay constant [s]
bsfc = 395;               % Brake specific fuel consumption [g/kWh]
rho_fuel = 0.75;          % Commercial gasoline density [kg/L]

n0   = 900;           % Idle speed [rpm]
t0   = 160;           % Idle torque [Nm]

na   = 2800;          % Turbo spool-up start threshold [rpm]
ta   = 220;           % Naturally aspirated torque before boost [Nm]

nb   = 4000;          % Peak torque RPM [rpm]
tb   = tmax_ice;      % Peak torque [Nm] (412 Nm)

nc   = 5500;          % Peak power RPM [rpm]
tc   = pmax_ice / (2 * pi * nc / 60); % Torque at peak power (~383 Nm)

nd   = 6700;          % Rev limiter [rpm]

% Generate RPM vector
dn = 1;
n_ice = 0:dn:nd;
t_ice = zeros(size(n_ice));
lag = zeros(size(n_ice));

for i = 1:length(n_ice)
    rpm = n_ice(i);
    if rpm <= n0
        t_ice(i) = t0; %idle
    elseif rpm <= na
        % Low RPM / Off-Boost Trait (Turbolag region)
        t_ice(i) = t0 + (ta - t0) * ((rpm - n0)/(na - n0));
        lag(i) = t_ice(i);
    elseif rpm <= nb
        % Turbo Spooling Trait (Boost buildup 0.0 -> 0.8 bar)
        t_ice(i) = ta + (tb - ta) * ((rpm - na)/(nb - na))^1.5;
    elseif rpm <= nc
        % Peak Torque to Peak Power Trait
        t_ice(i) = tb - (tb - tc) * ((rpm - nb)/(nc - nb));
    elseif rpm <= nd
        % High RPM Power Drop-off Trait
        t_ice(i) = tc * (nc / rpm)^1.2;
    end
end

p_ice = t_ice .* (n_ice * 2 * pi / 60); % ICE Power [W]

%% ELECTRIC MOTOR CURVE (Continuous S1 & Peak Overboost S2 Ratings)
pemax_em      = 50000;    % Continuous EM Power [W] (50 kW / 67 hp)
pemax_em_peak = 67500;    % Peak Overboost EM Power [W] (67.5 kW / 90.5 hp)
temax_em      = 230;      % Max EM Torque [Nm]

nea_em        = (pemax_em / temax_em) * (60 / (2*pi));       % Continuous base speed (~2075 rpm)
nea_em_peak   = (pemax_em_peak / temax_em) * (60 / (2*pi));  % Overboost base speed (~2800 rpm)
nemax_em      = 6700;                                        % Max motor speed [rpm]

% Equivalent LEMAd motor parameters (EM_model_v47b)
kt = 1.65; % Torque constant [Nm/Arms]

ne_em      = 0:dn:nemax_em;
te_em      = zeros(size(ne_em));
te_em_peak = zeros(size(ne_em));

for i = 1:length(ne_em)
    rpm = ne_em(i);
    % Continuous rating (50 kW)
    if rpm <= nea_em
        te_em(i) = temax_em;
    else
        te_em(i) = min(temax_em, pemax_em / (rpm * 2 * pi / 60));
    end
    
    % Peak overboost rating (67.5 kW, fills turbo lag up to 2800 rpm)
    if rpm <= nea_em_peak
        te_em_peak(i) = temax_em;
    else
        te_em_peak(i) = min(temax_em, pemax_em_peak / (rpm * 2 * pi / 60));
    end
end

pe_em      = te_em .* (ne_em * 2 * pi / 60);      % Continuous EM Power [W]
pe_em_peak = te_em_peak .* (ne_em * 2 * pi / 60); % Overboost EM Power [W]

%% HYBRID COMBINED PERFORMANCE
t_combined      = t_ice + te_em;
p_combined      = p_ice + pe_em;
t_combined_peak = t_ice + te_em_peak;
p_combined_peak = p_ice + pe_em_peak;

%% BRAKE BLENDING & CHASSIS DYNAMICS PARAMETERS
% --- Brake Bias ---
K_front_bias = 0.65;      % Front mechanical brake bias (65%)
K_rear_bias  = 0.35;      % Rear brake bias (35%)

% --- Electric Regen Limits ---
F_regen_max_base = 3500;  % Base regenerative braking force limit [N]

% --- SoC Derating Lookup Table ---
SoC_derate_bp  = [0, 0.80, 0.90, 1.0]; % Battery SoC breakpoints
SoC_derate_tbl = [1, 1,    0,    0  ]; % Scaling factor (1 = 100% regen, 0 = 0% regen)

% --- Low-Speed Cutoff Lookup Table (to allow mechanical stop) ---
v_cutoff_bp  = [0, 0.5, 1.4, 50];      % Speed breakpoints [m/s] (1.4 m/s ~ 5 km/h)
v_cutoff_tbl = [0, 0,   1,   1 ];      % Scaling factor

% --- Chassis & Longitudinal Dynamics Parameters ---
mu_road  = 0.9;           % Dry asphalt friction coefficient [-]
h_CoG    = 0.48;          % Center of Gravity Height [m]
a_front  = (1 - CoG_front) * wb; % Distance from Front Axle to CoG [m] (~1.329 m)
b_rear   = CoG_front * wb;       % Distance from Rear Axle to CoG [m] (~0.943 m)
J_wheels = 4 * 0.8;       % Total rotational inertia of 4 wheels [kg*m^2]
J_rotor = 0.025; % Rotor Inertia
J_drivetrain = (J_rotor + 0.22);% Rotor inertia + Engine flywheel inertia [kg*m^2]

% Equivalent mass calculation (for average gear ratio / 3rd gear)
i_avg = rt * gear_ratios(3); % Average gear ratio
m_eq = mg + (J_wheels / rw^2) + (J_drivetrain * i_avg^2 / rw^2); % Equivalent Mass [kg]

%% IMOLA Racetrack
s_pts = [0, 350, 450, 850, 950, 1350, 1450, 2000, 2400, 2500, 3000, 3100, 3800, 3900, 4100, 4600, 4700, 4909];
v_kph = [170, 220, 125, 195, 130, 175, 85, 135, 185, 105, 180, 110, 205, 100, 115, 195, 90, 170];
grade_pts = [0, 0, 0, 0, 0.0150, 0.0300, 0.0500, 0.0450, -0.0300, -0.0908, 0.0935, 0.0200, -0.0400, -0.0350, -0.0200, -0.0100, 0, 0]; % pu

s_track = 0:1:4909; % Every meter
v_track_kph = pchip(s_pts, v_kph, s_track); 
v_track_ms  = v_track_kph / 3.6;
grade_track = pchip(s_pts, grade_pts, s_track); % Slope 

distance_imola = s_track;
v_ref_imola = v_track_ms;
grade_imola = grade_track;
track_length = 4909;
%% PLOTTING RESULTS

% --- Figure 1: Vehicle Road Load Forces & Power ---
% figure('Name', 'Porsche 930 Road Load Analysis', 'Color', 'w');
% subplot(2,1,1);
% plot(vkph, fl, 'r-', vkph, fa, 'b--', vkph, frr, 'g--', vkph, fgrade, 'k:', 'LineWidth', 2);
% grid on;
% xlabel('Speed [km/h]'); ylabel('Force [N]');
% title('PORSCHE 930 TURBO - ROAD LOAD FORCES');
% legend('TOTAL LOAD', 'AERO DRAG', 'ROLLING RESIST.', 'GRADE', 'Location', 'northwest');
% 
% subplot(2,1,2);
% plot(vkph, pl/1000, 'r-', vkph, pa/1000, 'b--', vkph, prr/1000, 'g--', vkph, pgrade/1000, 'k:', 'LineWidth', 2);
% grid on;
% xlabel('Speed [km/h]'); ylabel('Power [kW]');
% title('PORSCHE 930 TURBO - ROAD LOAD POWER');
% legend('TOTAL POWER', 'AERO POWER', 'ROLLING POWER', 'GRADE POWER', 'Location', 'northwest');
% 
% % --- Figure 2: Specific Energy Consumption ---
% figure('Name', 'Specific Consumption', 'Color', 'w');
% plot(vkph(2:end), cs(2:end), 'b-', 'LineWidth', 2);
% grid on;
% xlabel('Speed [km/h]'); ylabel('Energy [Wh/km]');
% title('PORSCHE 930 TURBO - SPECIFIC ENERGY CONSUMPTION (ROAD LOAD)');
% 
% % --- Figure 3: ICE vs Motor vs Hybrid Combined Power & Torque (Overboost Mode) ---
% figure('Name', 'Porsche 930 Hybrid Powertrain Analysis (Overboost)', 'Color', 'w');
% subplot(2,1,1);
% plot(n_ice, t_ice, 'r--', 'LineWidth', 2); hold on;
% plot(ne_em, te_em_peak, 'b--', 'LineWidth', 2);
% plot(n_ice, t_combined_peak, 'k-', 'LineWidth', 2.5);
% grid on;
% title('POWERTRAIN TORQUE: TURBO LAG ELIMINATION');
% xlabel('Engine/Motor Speed [rpm]'); ylabel('Torque [Nm]');
% legend('ICE 3.3L Turbo (Stock Lag)', 'Electric Motor (67.5kW Overboost)', 'HYBRID COMBINED (Lag Filled)', 'Location', 'northeast');
% subplot(2,1,2);
% plot(n_ice, p_ice/1000, 'r--', 'LineWidth', 2); hold on;
% plot(ne_em, pe_em_peak/1000, 'b--', 'LineWidth', 2);
% plot(n_ice, p_combined_peak/1000, 'k-', 'LineWidth', 2.5);
% grid on;
% title('POWERTRAIN POWER CURVES');
% xlabel('Engine/Motor Speed [rpm]'); ylabel('Power [kW]');
% legend('ICE Power (300 hp)', 'Electric Motor (91 hp Overboost)', 'HYBRID TOTAL (391 hp)', 'Location', 'northwest');
% 
% 
% % --- Figure 4: Turbo Lag Mitigation Analysis (Continuous vs Overboost) ---
% figure('Name', 'Porsche 930 Hybrid lag analysis', 'Color', 'w');
% plot(n_ice, lag, 'r--', 'LineWidth', 1.8); hold on;
% plot(n_ice, t_ice, 'g-', 'LineWidth', 2.2);
% plot(ne_em, te_em, 'b:', 'LineWidth', 1.8);
% plot(ne_em, te_em_peak, 'b--', 'LineWidth', 2);
% plot(n_ice, t_combined, 'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'LineWidth', 2);
% plot(n_ice, t_combined_peak, 'k-', 'LineWidth', 2.8);
% grid on;
% title('PORSCHE 930 HYBRID - TURBO LAG OVERBOOST MITIGATION');
% xlabel('Engine/Motor Speed [rpm]'); ylabel('Torque [Nm]');
% legend('ICE Lag Region', ...
%        'ICE 3.3L Turbo (Stock)', ...
%        'EM Continuous (50 kW)', ...
%        'EM Overboost (67.5 kW Peak)', ...
%        'Hybrid Combined (50 kW Cont)', ...
%        'HYBRID COMBINED (Overboost Peak)', ...
%        'Location', 'northwest');

%%  Figure 5: ICE BSFC Efficiency Map & Active Load Point Shifting ---
% figure('Name', 'Porsche 930 BSFC Efficiency Map', 'Color', 'w', 'Position', [100 100 960 620]);
% 
% % 1. Griglia di calcolo 2D (Regime da 800 a 6700 rpm, Coppia da 20 a 450 Nm)
% [N_grid, T_grid] = meshgrid(linspace(800, 6700, 400), linspace(20, 450, 300));
% t_wot_interp = interp1(n_ice, t_ice, N_grid, 'linear', 'extrap');
% 
% % 2. Formulazione fisica della mappa BSFC (Sweet spot centrato a 4000 rpm & 380 Nm)
% R0 = 4000;          % [rpm] regime di massimo rendimento volumetrico
% T0 = 380;           % [Nm] coppia sweet spot (valvola a farfalla 100% aperta)
% bsfc_min = 252;     % [g/kWh] minimo consumo specifico (eta_th ~ 34%)
% 
% % Penalità di carico parzializzato (perdite di laminazione farfalla e pompaggio)
% t_ratio = max(T_grid / 380, 0.05);
% p_throttle = 132 * exp(-2.6 * t_ratio) + 40 * (1 - min(t_ratio, 1)).^1.8;
% 
% % Penalità di arricchimento miscela ad altissimo carico / WOT (lambda < 1.0 antidetonante)
% p_rich = 38 * max(0, (T_grid - 390) / 22).^2;
% 
% % Penalità di attrito e perdite termiche al variare del regime motore
% p_rpm = 24 * ((N_grid - R0) / 2100).^2 + 22 * max(0, (N_grid - 5200) / 1300).^2;
% 
% BSFC = bsfc_min + p_throttle + p_rich + p_rpm;
% BSFC(T_grid > t_wot_interp) = NaN; % Maschera i punti al di sopra della curva WOT
% 
% % 3. Palette colori personalizzata (Smeraldo/Verde -> Giallo -> Corallo/Rosso)
% cmap_custom = [
%     0.02, 0.59, 0.41;
%     0.06, 0.73, 0.51;
%     0.20, 0.83, 0.60;
%     0.43, 0.91, 0.72;
%     0.65, 0.95, 0.82;
%     0.99, 0.94, 0.54;
%     0.99, 0.88, 0.28;
%     0.98, 0.75, 0.14;
%     0.98, 0.57, 0.24;
%     0.97, 0.44, 0.44;
%     0.94, 0.27, 0.27
% ];
% cmap = interp1(linspace(1, size(cmap_custom,1), size(cmap_custom,1)), ...
%                cmap_custom, linspace(1, size(cmap_custom,1), 128));
% 
% levels = [253, 260, 272, 290, 315, 350, 395, 450, 520];
% contourf(N_grid, T_grid, BSFC, levels, 'LineStyle', 'none'); hold on;
% colormap(cmap);
% clim([253, 520]);
% 
% % Linee di livello con etichette del valore BSFC
% [C_lines, h_lines] = contour(N_grid, T_grid, BSFC, levels, 'LineColor', [0.12 0.16 0.23], 'LineWidth', 0.85);
% clabel(C_lines, h_lines, 'FontSize', 8, 'Color', [0.06 0.09 0.16], 'LabelSpacing', 350);
% 
% % 4. Zona inaccessibile oltre il limite WOT (sfondo grigio protettivo)
% idx_valid = (n_ice >= 800) & (n_ice <= 6700);
% n_sub = n_ice(idx_valid);
% t_sub = t_ice(idx_valid);
% fill([n_sub, fliplr(n_sub)], [t_sub, 450 * ones(size(t_sub))], [0.89 0.92 0.95], ...
%      'EdgeColor', 'none', 'FaceAlpha', 0.92);
% 
% % Curva WOT Stock (Porsche 930 Turbo 3.3L)
% h_wot = plot(n_ice, t_ice, 'Color', [0.06 0.09 0.16], 'LineWidth', 3.2);
% 
% % 5. Curve di iso-potenza motore (50, 100, 150, 200 kW)
% rpm_iso = linspace(1000, 6700, 250);
% for p_kw = [50, 100, 150, 200]
%     t_iso = (p_kw * 1000) ./ (rpm_iso * 2 * pi / 60);
%     t_lim_iso = interp1(n_ice, t_ice, rpm_iso, 'linear', 'extrap');
%     valid = (t_iso <= t_lim_iso) & (t_iso >= 25);
%     if any(valid)
%         plot(rpm_iso(valid), t_iso(valid), ':', 'Color', [0.39 0.45 0.55], 'LineWidth', 1.2);
%         idx_lbl = find(valid, 1, 'last');
%         text(rpm_iso(idx_lbl)-280, t_iso(idx_lbl)+6, sprintf('%d kW', p_kw), ...
%             'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.30 0.35 0.45]);
%     end
% end
% 
% % 6. Evidenziazione Zona E-Mode (< 2500 rpm: spegnimento ICE)
% patch([800 2500 2500 800], [0 0 450 450], [0.23 0.51 0.96], 'FaceAlpha', 0.08, 'EdgeColor', 'none');
% text(1650, 420, sprintf('E-MODE ZONE (< 2500 rpm)\nICE Off • Zero Throttling Losses'), ...
%     'HorizontalAlignment', 'center', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.12 0.25 0.69], ...
%     'BackgroundColor', [0.94 0.96 1.0], 'EdgeColor', [0.58 0.77 0.99], 'Margin', 3);
% 
% % 7. Vettore di Active Load Point Shifting (150 Nm -> 380 Nm @ 4000 rpm)
% shift_rpm = 4000;
% demand_t  = 150;
% shifted_t = 380;
% 
% % Vettore verde con freccia
% line([shift_rpm shift_rpm], [demand_t + 8, shifted_t - 8], 'Color', [0.02 0.59 0.41], 'LineWidth', 3.2);
% plot(shift_rpm, shifted_t - 8, '^', 'MarkerFaceColor', [0.02 0.59 0.41], 'MarkerEdgeColor', [0.02 0.59 0.41], 'MarkerSize', 9);
% 
% % Punti operativi
% h_shift  = plot(shift_rpm, shifted_t, 'o', 'MarkerFaceColor', [0.02 0.47 0.34], 'MarkerEdgeColor', [0.02 0.31 0.23], 'MarkerSize', 11, 'LineWidth', 2);
% h_demand = plot(shift_rpm, demand_t,  'o', 'MarkerFaceColor', [0.86 0.15 0.15], 'MarkerEdgeColor', [0.6 0.1 0.1],   'MarkerSize', 9,  'LineWidth', 1.5);
% 
% % Box descrittivi
% text(shift_rpm - 150, demand_t, sprintf('Part-Load Demand\n150 Nm (BSFC ~335 g/kWh)\nHigh Throttling Losses'), ...
%     'HorizontalAlignment', 'right', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.73 0.11 0.11], ...
%     'BackgroundColor', [1.0 0.95 0.95], 'EdgeColor', [0.94 0.27 0.27], 'Margin', 3);
% 
% text(shift_rpm + 150, 265, sprintf('Active Load Point Shift\n\\DeltaT = +230 Nm to Battery\n(EM Generator @ 230 Nm Rated)'), ...
%     'HorizontalAlignment', 'left', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.02 0.37 0.27], ...
%     'BackgroundColor', [0.94 0.99 0.96], 'EdgeColor', [0.13 0.77 0.37], 'Margin', 4);
% 
% text(4700, 360, sprintf('Optimal BSFC Island (380 Nm)\n\\eta_{th} \\approx 34%% (252 g/kWh)\nThrottle Wide Open • Optimal Boost'), ...
%     'HorizontalAlignment', 'left', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.02 0.37 0.27], ...
%     'BackgroundColor', [0.93 0.99 0.96], 'EdgeColor', [0.06 0.73 0.51], 'Margin', 4);
% 
% plot([shift_rpm + 50, 4680], [shifted_t, 360], '--', 'Color', [0.02 0.47 0.34], 'LineWidth', 1.4);
% 
% % 8. Formattazione assi e legenda
% grid on;
% set(gca, 'Layer', 'top'); % Mantiene griglia e assi nitidi sopra i contorni
% xlim([800 6700]);
% ylim([0 450]);
% xlabel('Engine Speed [rpm]', 'FontSize', 11, 'FontWeight', 'bold');
% ylabel('Engine Brake Torque [Nm]', 'FontSize', 11, 'FontWeight', 'bold');
% title('PORSCHE 930 TURBO 3.3L — BSFC EFFICIENCY MAP & LOAD POINT SHIFTING', ...
%       'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.06 0.13 0.26]);
% 
% c = colorbar;
% ylabel(c, 'BSFC [g/kWh]', 'FontSize', 10, 'FontWeight', 'bold');
% 
% legend([h_wot, h_shift, h_demand], ...
%        {'ICE WOT Limit (Porsche 930 3.3L)', 'Shifted Point: 380 Nm (Sweet Spot)', 'Vehicle Demand (150 Nm)'}, ...
%        'Location', 'southwest', 'FontSize', 8.5);