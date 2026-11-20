function R=ModeloDinamico_M1(P,carpeta)
% M1: se simula junto con el otro eje lateral; comparten el ladrillo.
if nargin<1, P=ParametrosMaquina("prueba"); end
if nargin<2, carpeta=fullfile(fileparts(mfilename('fullpath')),"Resultados_"+P.modo); end
R=SimularModeloMecanico("sujecion",P,carpeta);
end
