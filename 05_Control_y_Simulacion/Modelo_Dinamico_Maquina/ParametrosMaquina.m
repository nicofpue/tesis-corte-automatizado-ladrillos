function P=ParametrosMaquina(modo)
% Todos los calculos internos usan SI: m, s, kg, N, rad; PWM en %.
% 'prueba': supuestos para ejecutar; 'real': bloquea calculo si faltan mediciones.
if nargin<1, modo="prueba"; end
assert(any(string(modo)==["prueba","real"]),'Modo invalido');
P.modo=string(modo); P.validado=false; P.Ts=0.005; P.subpasos=10;
P.g=9.81; P.v_friccion=0.001; % regularizacion de Coulomb, no friccion estatica exacta
P.sujecion.m_ladrillo=2.5; % kg: valor documentado; confirmar pieza de ensayo
P.sujecion.mu=0.20; % PRUEBA, ladrillo/plataforma, NO coeficiente de las guias
P.sujecion.b_ladrillo=1; % PRUEBA N*s/m
P.sujecion.holgura_inicial=0.002; % m, PRUEBA, cada lado
P.sujecion.vel_aproximacion=0.001; % m/s, objetivo de prueba
P.sujecion.tmax_compresion=20; % s, timeout de simulacion
P.sujecion.confirmacion_s=0.05; % ambos switches activos para cambiar de etapa
P.sujecion.desplazamiento=0.020; % m, posicionamiento lateral de prueba
P.sujecion.t_movimiento=4; P.sujecion.t_mantenimiento=2;
for i=1:2 % M1 y M2 (M3 no se incluye: opera en lazo abierto)
    A.nombre="M"+i; A.N=1; A.p=0.008; A.CPR=1000;
    A.m_carro=1; % kg, PRUEBA, sin ladrillo
    A.J_motor=11.9e-6; % kg*m^2; constante del script anterior, por confirmar
    A.J_husillo=1e-6; A.J_reductor_salida=0; % PRUEBA
    A.b=20; A.F_coulomb=0.5; % PRUEBA: resistencia equivalente del eje
    A.Ku=2; % N por punto %PWM EFECTIVO: ganancia de fuerza equivalente, NO Kv
    A.tau_driver=0.005; % s, PRUEBA: retardo de primer orden del esfuerzo
    A.F_max=60; % N, PRUEBA: limite de esfuerzo equivalente, NO capacidad certificada
    A.PWM_dead=5; A.PWM_max=35; % PRUEBA, comprobar por sentido
    A.eta=0.40; % hipotesis de diseno husillo; no medicion
    A.k=1000; A.c_resorte=2; A.F_precarga=0; % N/m, N*s/m, N; PRUEBA
    A.delta_switch=0.005; A.delta_max=0.012; % m, PRUEBA; no usar 38 mm sin verificar
    A.q_min=-0.040; A.q_max=0.080; % m, limites de simulacion de prueba
    A.Kp=5000; A.Ki=1000; A.Kd=200; A.Tf=0.020; % %/m, %/(m*s), %s/m
    A.origen="Supuestos de prueba; NO parametros identificados";
    if modo=="real"
        % COMPLETAR con datos comprobados. No sustituir NaN por valores sin trazabilidad.
        A.J_motor=NaN; A.J_husillo=NaN; A.b=NaN; A.F_coulomb=NaN;
        A.Ku=NaN; A.tau_driver=NaN; A.F_max=NaN; A.PWM_dead=NaN;
        A.m_carro=NaN; A.k=NaN; A.c_resorte=NaN;
        A.F_precarga=NaN; A.delta_switch=NaN; A.delta_max=NaN;
        A.origen="Plantilla real: completar y documentar mediciones";
    end
    P.motor(i)=A;
end
if modo=="real"
    P.sujecion.mu=NaN; P.sujecion.b_ladrillo=NaN;
end
% La decodificacion CPR=1000 debe verificarse. No se han identificado driver,
% rozamiento, rigidez ni limites mediante los registros del ZIP.
end
