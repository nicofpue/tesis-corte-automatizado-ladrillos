function VerificarModelos(carpeta)
% Pruebas con parametros de ejemplo; NO sustituye validacion experimental.
if nargin<1, carpeta=fullfile(tempdir,'VerificacionModelosMecanicos'); end
vis=get(groot,'DefaultFigureVisible');
restaurar=onCleanup(@()set(groot,'DefaultFigureVisible',vis));
set(groot,'DefaultFigureVisible','off');
P=ParametrosMaquina("prueba");
R=SimularModeloMecanico("sujecion",P,fullfile(carpeta,'base'));
assert(isfinite(R.t_cambio),'La simulacion no completo la compresion.');
assert(R.motivo=="Compresion y traslado completados",'Terminacion inesperada.');
post=R.M1.Etapa==2;
assert(all(R.M1.Compresion_mm(post)>0) && all(R.M2.Compresion_mm(post)>0), ...
    'Se perdio el contacto durante traslado.');
B=R.ladrillo;
residuo=P.sujecion.m_ladrillo*B.a_m_s2- ...
    (B.F_M1_N-B.F_M2_N-B.F_rozamiento_N);
assert(max(abs(residuo))<1e-9,'Fallo del balance de fuerzas del ladrillo.');
for i=1:2
    T=R.("M"+i); A=P.motor(i);
    if i==1, vd=(T.v_mm_s-B.v_mm_s)/1000;
    else, vd=(B.v_mm_s-T.v_mm_s)/1000; end
    F=max(0,A.F_precarga+A.k*T.Compresion_mm/1000+A.c_resorte*vd);
    F(T.Compresion_mm<=0)=0;
    assert(max(abs(F-T.F_contacto_N))<1e-8,'Fallo de ley elastica/contacto.');
    assert(T.Cuentas_encoder(1)==0,'El encoder debe comenzar en cero relativo.');
end
P.Ts=P.Ts/2;
H=SimularModeloMecanico("sujecion",P,fullfile(carpeta,'paso_mitad'));
assert(abs(R.ladrillo.y_mm(end)-H.ladrillo.y_mm(end))<0.1, ...
    'Sensibilidad temporal lateral mayor que tolerancia numerica de prueba.');
fprintf('PRUEBAS NUMERICAS OK. No es validacion de la maquina real.\n');
end
