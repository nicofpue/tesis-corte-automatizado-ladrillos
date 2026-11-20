% Entrada principal: no conecta ni mueve motores reales.
% Simula la sujecion: M1, M2 y el ladrillo.
% NOTA: ParametrosMaquina("prueba") usa parametros SUPUESTOS (no identificados).
% Los resultados son ilustrativos y no validan la maquina real.
clear; clc; close all;
base=fileparts(mfilename('fullpath')); addpath(base);
P=ParametrosMaquina("prueba"); % Cambiar a real SOLO tras completar parametros
carpeta=fullfile(base,"Resultados_"+P.modo);
R12=ModeloDinamico_M1(P,carpeta); % genera M1 y M2, sin duplicar la masa del ladrillo
disp(R12.motivo);
fprintf('Simulacion con parametros %s. No son mediciones nuevas.\n',P.modo);
