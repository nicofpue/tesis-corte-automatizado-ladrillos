function R=ModeloDinamico_M3(P,carpeta)
% M3: avance y retorno con reductor 10:1, sin resorte.
if nargin<1, P=ParametrosMaquina("prueba"); end
if nargin<2, carpeta=fullfile(fileparts(mfilename('fullpath')),"Resultados_"+P.modo); end
R=SimularModeloMecanico("avance",P,carpeta);
end
