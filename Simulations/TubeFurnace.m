close all
clear
clc

%% UBC HackerFab - Tube Furnace Simulation
% Author: Jason Kwok
% Created: October 9th, 2025
% Last Modified: October 11th, 2025


%% Constants
stefan = 5.67*10^-8;    % W/m^2K^4
g = 9.81;               % m/s^2
ft_to_m = 0.3048;       % m/ft
in_to_mm = 25.4;        % mm/in
oz_to_g = 28.35;        % g/oz

% Quartz
q_emis = 0.8;           % Emissivity
q_density = 2200;       % kg/m^3
q_heat_cap = 670;       % Heat capacity, J/kgK

% Air
a_density = 1.225;      % kg/m^3
a_heat_cap = 1005;      % Heat capacity, J/kgK


% Research into the heating curves for existing tub furnaces
% compare to this, target time power etc
% 15 x 15mm silicon wafers


%% Parameters

% Quartz tube
q_inner_dia = 20;       % mm
q_thickness = 1;        % mm
q_length = 1 * ft_to_m; % m
q_weight = 6.25;        % oz

% Insulation
i_thickness = 100;      % mm
i_therm_cond = 0.1;     % Thermal conductivity, W/mK

% Heating Wire
w_diam = 0.51;          % mm
w_resist = 2 / ft_to_m; % Resistivity, ohm/m
w_loop_space = 5;       % mm - UNUSED
w_length = 100;         % cm


%% Set Inputs
current = 15;           % Max current, Amps
ambient_temp = 25;      % Celcius
heat_eff = 0.85;        % Electrical power to heat efficiency


%% Calculate needed variables

% Quartz
q_mass = q_weight*oz_to_g/1000;                            % kg
q_inner_sa = q_length * pi * (q_inner_dia / 2000);         % m^2
q_inner_v = q_length * pi * (q_inner_dia / 2000) ^ 2;      % m^3

% Heating wire
w_tot_resist = w_resist * w_length / 100;                  % Ohm
w_power = heat_eff * current ^ 2 * w_resist;               % W

% Air
a_inner_m = q_inner_v * a_density;                         % kg

% Insulation
i_resist_cond = log((q_inner_dia/2 + i_thickness) / (q_inner_dia/2)) / (2*pi*i_therm_cond*q_length); % K/W


%% PID Tuning
k_p = 0.5;
k_i = 0;
k_d = 0;
asd = w_resist * 8;

%% Run and save the simulation
Output=sim("TubeFurnaceSim.slx");

figure()
plot(Output.q_t.Time, Output.q_t.Data);
ylabel('Temperature [C]')
xlabel('Time [s]')
title(['Tube Furnace Heating Curve - Max ', num2str(current), ' Amps'])

hold on

plot(Output.a_t.Time, Output.a_t.Data);

legend('Tube', 'Air');
print(['Tube Furnace Heating Curve - Max ', num2str(current), ' Amps'], '-dpng');

hold off

figure()
plot(Output.current.Time, Output.current.Data);
ylabel('Current [A]');
xlabel('Time [s]')
ylim([0, current + 1]);
title('PID Output for Current')
