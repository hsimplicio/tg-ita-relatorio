clear; clc; close all;

% Adicione o caminho para a classe TrajectoryProblem
addpath('..');

% Crie uma instância do problema
nx = 2;  % Número de estados
nu = 1;  % Número de controles
problem = TrajectoryProblem(nx, nu);

% Defina as propriedades básicas do problema
problem.setTimeBoundaryConditions(0, 1);  % Intervalo de tempo [0,1]
problem.setBoundaryConditions(x0, xF);  % Condições iniciais e finais
problem.setStateBounds(xLow, xUpp);  % Limites de estado
problem.setControlBounds(uLow, uUpp);  % Limites de controle

% Defina as funções necessárias
problem.setDynamics(@systemDynamics);  % Dinâmica do sistema
problem.setObjective(@boundaryObj, @pathObj);  % Funções objetivo
problem.setConstraints(@boundaryConst, @pathConst);  % Restrições opcionais

% Defina as opções do solver
options = optimoptions('fmincon', 'Display', 'iter');
nGrid = [50, 100, 200];  % Várias iterações com refinamento de malha
problem.setSolverOptions(options, nGrid);

% Resolva o problema
solution = problem.solveWithTrapezoidalCollocation();