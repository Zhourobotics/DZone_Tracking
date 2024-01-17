% Script for simulating multi-quad collision avoidance using mpc
addpath("/home/carl/Github/resilient-target-tracking/chance-constraint-resilient-target-tracking/dynamics");

%% Clean workspace
clear
close all
clearvars
clearvars -global
clc

global config
configFile = 'config.json'; 
configHandler = fopen(configFile);
raw = fread(configHandler, +inf);
str = char(raw'); % Transformation
fclose(configHandler);
config = jsondecode(str);


%% Initialization
initialize;

%% Generate solver
if getNewSolver 
    mpc_generator_chance;
end
