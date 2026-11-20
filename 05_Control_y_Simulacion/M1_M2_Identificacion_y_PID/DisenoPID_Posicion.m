%% ============================================================
% DISENO Y COMPROBACION DEL PID DE POSICION DE UN EJE LINEAL
%
% Este archivo DISENA el controlador. El ESP32 EJECUTA el controlador.
% Visual Studio Code + PlatformIO es el entorno; Arduino usa C/C++.
% No necesitas ejecutar MATLAB ni Python dentro del ESP32.
%
% ENTRADA : Modelo_Eje1.mat (lo genera SimulacionModeloDinamico_Revisado.m).
% ORIGEN DE LAS GANANCIAS DEL FIRMWARE (M1 y M2): este script con
%   Kv=3.99 (mm/s)/%PWM, tau=0.020 s, T_EST_OBJ=1.5 s, ZETA=0.90
%   -> Kp=1.6468, Ki=2.6383, Kd=0.076832 (config_eje.h).
% Ts, PWM_MAX y Tf estan fijados a los valores del firmware.
% SALIDA  : Kp, Ki, Kd, Tf, Ts y coeficientes discretos por eje.
%
% Misma planta que el archivo de identificacion:
%   Gv(s) = Kv/(tau*s+1)       [mm/s por punto porcentual de PWM]
%   Gx(s) = Kv/[s*(tau*s+1)]   [mm por punto porcentual de PWM]
%
% El PID entrega PWM EFECTIVO con signo, antes de compensar zona muerta.
% El signo positivo SIEMPRE debe aumentar la posicion medida.
% La correspondencia signo -> pin DIR se configura en el firmware.
%
% SECCIONES:
% 0) Datos editables y carga de UN modelo, sin inventar otros motores.
% 1) Planta y unidades.
% 2) Calculo transparente por polos (o pidtune opcional).
% 3) Forma discreta EXACTA que debe usar el firmware.
% 4) Comprobacion lineal discreta con derivada sobre la medicion.
% 5) Simulacion con saturacion, zona muerta, encoder y anti-windup.
% 6) Graficas, indicadores y exportacion.
%
% REQUIERE: MATLAB R2016b o posterior + Control System Toolbox.
% Las funciones locales deben permanecer al FINAL del archivo.
% Este script NO conecta ni mueve motores.
%% ============================================================
clear; clc; close all;

%% ============================================================
% 0. DATOS QUE TU MODIFICAS
%% ============================================================
BASE_DIR = fileparts(mfilename('fullpath'));
ARCHIVO_MODELO = fullfile(BASE_DIR,'Modelo_Eje1.mat');

% "archivo": usar identificacion exportada (recomendado).
% "ejemplo": probar el codigo con parametros previos NO validados aqui.
FUENTE_MODELO = "archivo";
METODO = "polos"; % "polos" (explicito) o "pidtune" (ajuste automatico)

% Objetivos de diseno iniciales, NO garantias del mecanismo real.
T_EST_OBJ = 1.5;       % s: tiempo de establecimiento aproximado al 2%
ZETA = 0.90;          % amortiguamiento de los polos dominantes (0<zeta<=1)
FACTOR_POLO = 5;      % tercer polo al menos 5 veces mas rapido que wn
FACTOR_FILTRO = 10;   % frecuencia de filtro / frecuencia del polo mas rapido
MARGEN_FASE_OBJ = 65; % grados; utilizado SOLO por la opcion pidtune

Ts = 0.010;          % s: periodo del lazo del firmware (controlPeriodUs = 10000)
% IMPORTANTE: NO es el periodo de la portadora PWM (p.ej., 20 kHz).
% Debe comprobarse que la tarea cumple Ts con poco jitter y sin bloqueos.
PWM_MAX = 20;        % % fisico: techo del PID en el firmware (maxPwm = 20)
% No es una especificacion electrica: verificar con tus pruebas y carga.
TOL_FINAL_MM = 0.10; % mm: criterio de evaluacion, NO apaga automaticamente PID

% Pruebas simuladas. No son instrucciones para ejecutar en el prototipo.
PASO_MM = 10;        % desplazamiento de la prueba de escalon
RECORRIDO_MM = 60;   % desplazamiento de la trayectoria suave (recorrido M1 = 68 mm)
TIEMPO_TRAY_S = 5;   % duracion de la trayectoria de movimiento
TIEMPO_SIM_S = 9;    % incluye tiempo final para observar el asentamiento
TF_FIRMWARE = 0.03;  % s: filtro derivativo del firmware (derivativeFilterS); [] = calcularlo
CUANTIZAR_ENCODER = true;
EXPORTAR = true;
MOSTRAR_GRAFICAS = true;

if FUENTE_MODELO == "archivo"
    if ~isfile(ARCHIVO_MODELO)
        error(['Falta %s. Ejecuta primero SimulacionModeloDinamico_Revisado.m. ' ...
               'Para inspeccionar este script sin TXT, elige FUENTE_MODELO="ejemplo".'], ...
               ARCHIVO_MODELO);
    end
    cargado = load(ARCHIVO_MODELO,'Modelo');
    assert(isfield(cargado,'Modelo'),'El MAT no contiene la estructura Modelo.');
    Modelo = cargado.Modelo;
elseif FUENTE_MODELO == "ejemplo"
    Modelo = struct('eje',"Eje1_EJEMPLO",'Kv',3.99,'tau',0.020, ...
        'PWM_dead',5.55,'mm_por_cuenta',0.008,'DIR_MODELO',-1, ...
        'fuente',"EJEMPLO: valores usados para las ganancias del firmware", ...
        'validacion_aprobada',false);
    warning('MODO EJEMPLO: resultados ilustrativos; no son ganancias validadas.');
else
    error('FUENTE_MODELO debe ser "archivo" o "ejemplo".');
end

campos = {'eje','Kv','tau','PWM_dead','mm_por_cuenta'};
for k = 1:numel(campos)
    assert(isfield(Modelo,campos{k}),'Falta campo Modelo.%s.',campos{k});
end
Kv = Modelo.Kv; tau = Modelo.tau; PWM_dead = Modelo.PWM_dead;
q = Modelo.mm_por_cuenta;
validateattributes(Kv,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(tau,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(PWM_dead,{'numeric'},{'scalar','real','finite','>=',0,'<',100});
validateattributes(q,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(Ts,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(PWM_MAX,{'numeric'},{'scalar','real','finite','>',PWM_dead,'<=',100});
assert(T_EST_OBJ>0 && ZETA>0 && ZETA<=1 && FACTOR_POLO>=3 && FACTOR_FILTRO>=3, ...
    'Revisar especificaciones de los polos y filtro.');
assert(PASO_MM>0 && RECORRIDO_MM>0 && TIEMPO_TRAY_S>0 && TIEMPO_SIM_S>TIEMPO_TRAY_S+1, ...
    'Revisar desplazamientos y tiempos de simulacion.');
assert(TOL_FINAL_MM>=q/2,'Tolerancia incompatible con resolucion del encoder.');
if Ts > tau/10
    warning('Ts > tau/10: revisar muestreo; no conservar ganancias sin comprobarlo.');
end
if ~isfield(Modelo,'validacion_aprobada') || ~Modelo.validacion_aprobada
    warning('Modelo sin validacion experimental aprobada: ganancias candidatas.');
end
fprintf('\n========================================\n');
fprintf('DISENO DEL EJE: %s\n',char(Modelo.eje));
fprintf('Kv=%.8f | tau=%.8f s | zona muerta=%.5f %%\n',Kv,tau,PWM_dead);
fprintf('Resolucion=%.8f mm/cuenta | Ts=%.6f s\n',q,Ts);
if isfield(Modelo,'DIR_MODELO') && Modelo.DIR_MODELO~=0
    warning('Modelo de UN sentido (DIR=%d). Comprobar tambien el retorno.',Modelo.DIR_MODELO);
end

%% ============================================================
% 1. PLANTA LINEAL Y LIMITES
%
% Dinamica equivalente:
%     x_dot = v
%     v_dot = -v/tau + Kv*u_efectivo/tau
%
% Kv NO es la constante electrica rpm/V del motor.
% tau NO es el tiempo de recorrido ni el tiempo de establecimiento deseado.
%% ============================================================
s = tf('s');
Gx = Kv/(s*(tau*s+1));
Gd = c2d(Gx,Ts,'zoh'); % mando constante entre dos actualizaciones del ESP32
U_MAX = PWM_MAX-PWM_dead;
% Al compensar zona muerta solo quedan estos puntos porcentuales efectivos.

%% ============================================================
% 2. CALCULAR Kp, Ki Y Kd
%
% PID ideal paralelo:
%     C(s) = Kp + Ki/s + Kd*s
%
% Polinomio caracteristico del lazo ideal:
%     tau*s^3 + (1+Kv*Kd)*s^2 + Kv*Kp*s + Kv*Ki
%
% Elegimos el polinomio deseado:
%     tau*(s+p3)*(s^2 + 2*zeta*wn*s + wn^2)
%
% Igualando coeficientes:
%     Kp = tau*(wn^2 + 2*zeta*wn*p3)/Kv
%     Ki = tau*p3*wn^2/Kv
%     Kd = (tau*(2*zeta*wn+p3)-1)/Kv
%
% wn ~= 4/(zeta*T_EST_OBJ) es una aproximacion de segundo orden.
% Tf, muestreo y ceros modifican la respuesta: SIMULAR y medir despues.
% El criterio p3>=1.2/tau mantiene Kd positivo para esta planta positiva.
% No significa que existan ganancias unicas ni optimas para cualquier carga.
%% ============================================================
wn = 4/(ZETA*T_EST_OBJ); % rad/s
p3 = max(FACTOR_POLO*wn,1.2/tau); % rad/s
Kp = tau*(wn^2+2*ZETA*wn*p3)/Kv;
Ki = tau*p3*wn^2/Kv;
Kd = (tau*(2*ZETA*wn+p3)-1)/Kv;
Tf = 1/(FACTOR_FILTRO*max(p3,wn)); % s: filtro real de la derivada
if ~isempty(TF_FIRMWARE), Tf = TF_FIRMWARE; end % usar el filtro implementado

% Controlador filtrado: C(s)=Kp+Ki/s+Kd*s/(Tf*s+1).
% Usamos Backward Euler para integral y filtro derivativo.
% Si se elige pidtune, esta plantilla conserva la misma forma discreta.
Cd = pid(Kp,Ki,Kd,Tf,Ts,'IFormula','BackwardEuler','DFormula','BackwardEuler');
if METODO == "pidtune"
    opciones = pidtuneOptions('PhaseMargin',MARGEN_FASE_OBJ);
    [Cd,infoTune] = pidtune(Gd,Cd,wn,opciones);
    Kp=Cd.Kp; Ki=Cd.Ki; Kd=Cd.Kd; Tf=Cd.Tf;
    disp(infoTune);
elseif METODO ~= "polos"
    error('METODO debe ser "polos" o "pidtune".');
end
assert(all(isfinite([Kp Ki Kd Tf])) && all([Kp Ki Kd]>=0) && Tf>0, ...
    'Ganancias o filtro no validos. Revisar modelo/especificaciones.');
if p3*Ts>0.2 || wn*Ts>0.1
    warning('Polos deseados rapidos para Ts. Aumentar T_EST_OBJ o reducir Ts.');
end

%% ============================================================
% 3. COEFICIENTES EXACTOS PARA ARDUINO / PLATFORMIO
%
% Entrada de cada iteracion: referencia r[k] y posicion x[k], en mm.
% Estados PRIVADOS por motor: Iprev, Dprev, xprev.
%
% e[k]     = r[k]-x[k]
% P[k]     = Kp*e[k]
% Itrial   = Iprev + Ki*Ts*e[k]
% D[k]     = A_D*Dprev - B_D*(x[k]-xprev)
% uTrial   = P[k]+Itrial+D[k]
%
% Derivada sobre MEDICION: evita el golpe derivativo de un escalon de r.
% La realimentacion conserva el denominador del PID convencional.
% La respuesta a referencia tiene otro numerador y puede otro sobreimpulso.
%
% Anti-windup por integracion condicional:
% Si uTrial>U_MAX y e>0, o uTrial<-U_MAX y e<0: conservar Iprev.
% Si no: aceptar Itrial, limitado ademas a [-I_MAX,+I_MAX].
% Finalmente u=limitar(P+I+D,-U_MAX,+U_MAX).
%
% Compensacion estatica de zona muerta:
%   u=0  -> PWM fisico=0
%   u~=0 -> PWM fisico=sign(u)*(abs(u)+PWM_dead)
% Verificar que el driver responda a esta aproximacion cerca de cero.
%
% NO multiplicar por Ts otra vez si usas KI_PASO=Ki*Ts.
% NO dividir por Ts otra vez si usas B_D=Kd/(Tf+Ts).
% Si tu libreria PID usa otra forma, adaptar; no copiar coeficientes a ciegas.
%% ============================================================
KI_PASO = Ki*Ts;
A_D = Tf/(Tf+Ts);
B_D = Kd/(Tf+Ts);
I_MAX = U_MAX; % limite adicional en UNIDADES DE SALIDA, %PWM efectivo

fprintf('\n========================================\n');
fprintf('GANANCIAS CANDIDATAS (UNIDADES EXPLICITAS)\n');
fprintf('Kp = %.10g %%PWM/mm\n',Kp);
fprintf('Ki = %.10g %%PWM/(mm*s)\n',Ki);
fprintf('Kd = %.10g %%PWM*s/mm\n',Kd);
fprintf('Tf = %.10g s\n',Tf);
fprintf('Ts = %.10g s | frecuencia lazo = %.3f Hz\n',Ts,1/Ts);
fprintf('Ki*Ts = %.10g | A_D = %.10g | B_D = %.10g\n',KI_PASO,A_D,B_D);
fprintf('PWM fisico maximo = %.3f %% | efectivo maximo = %.3f %%\n',PWM_MAX,U_MAX);
fprintf('Para salida duty 0..1: dividir Kp,Ki,Kd y limites por 100.\n');
fprintf('Para error en cuentas: Kp,Ki,Kd se multiplican por mm/cuenta.\n');
fprintf('Si usas mm y %% como aqui, NO realizar esas conversiones.\n');

%% ============================================================
% 4. VERIFICAR ESTABILIDAD EN TIEMPO DISCRETO
%
% PID implementado: u = (Kp+Ki*integrador)*r - Cd*x.
% El filtro solo deriva x. Usamos el numerador correcto para la referencia.
% Comprobar estabilidad ANTES de simular movimiento.
% No incluye retraso de una tarea adicional; si existe, modelarlo aparte.
%% ============================================================
Cpi = pid(Kp,Ki,0,0,Ts,'IFormula','BackwardEuler');
L = Cd*Gd;
T_ref = feedback(Gd,Cd)*Cpi;
T_feedback = feedback(L,1);
polos_discretos = pole(T_feedback);
if ~isstable(T_feedback)
    error('Lazo discreto INESTABLE. Reducir rapidez o revisar muestreo/modelo.');
end
[GM,PM,~,Wcp] = margin(L);
fprintf('\nESTABILIDAD LINEAL DISCRETA\n');
fprintf('Maximo modulo polos = %.8f (debe ser <1)\n',max(abs(polos_discretos)));
fprintf('Margen de fase = %.3f grados | cruce = %.5f rad/s\n',PM,Wcp);
fprintf('Margen de ganancia = %.3f dB\n',20*log10(GM));
if isfinite(PM) && PM<45
    warning('Margen de fase menor de 45 grados: revisar robustez antes de implementar.');
end

%% ============================================================
% 5. SIMULAR EL MISMO ALGORITMO QUE USARIA EL FIRMWARE
%
% Incluye: cero orden, encoder cuantizado opcional, saturacion fisica,
% zona muerta estatica, filtro derivativo e integral condicional.
% La compensacion usa la zona muerta nominal; la planta puede tener otra.
% NO modela: friccion real, elasticidad, limites fisicos, fuerza de corte,
% corriente, frenado del driver, holgura ni seguridad de la maquina.
%
% Casos adicionales = sensibilidad ILUSTRATIVA, no incertidumbre medida.
% El modelo inverso de zona muerta es idealizado, sobre todo cerca de cero.
%% ============================================================
P = struct('Kp',Kp,'Ki',Ki,'Kd',Kd,'Tf',Tf,'Ts',Ts, ...
    'KI_PASO',KI_PASO,'A_D',A_D,'B_D',B_D,'I_MAX',I_MAX, ...
    'PWM_MAX',PWM_MAX,'PWM_dead',PWM_dead,'U_MAX',U_MAX, ...
    'q',q,'cuantizar',CUANTIZAR_ENCODER);
t = (0:ceil(TIEMPO_SIM_S/Ts))'*Ts;
tInicio = 0.5;
rPaso = PASO_MM*double(t>=tInicio);

% Polinomio quintico: arranca y termina con velocidad/aceleracion nulas.
% Recorrer 150 mm en 5 s implica velocidad MEDIA=30 mm/s,
% pero este perfil tiene velocidad MAXIMA=1.875*150/5=56.25 mm/s.
a = min(max((t-tInicio)/TIEMPO_TRAY_S,0),1);
rTray = RECORRIDO_MM*(10*a.^3-15*a.^4+6*a.^5);
vPico = 1.875*RECORRIDO_MM/TIEMPO_TRAY_S;
vMaxModelo = Kv*U_MAX;
fprintf('\nTRAYECTORIA\n');
fprintf('Velocidad pico requerida = %.4f mm/s\n',vPico);
fprintf('Velocidad estacionaria max. del modelo = %.4f mm/s\n',vMaxModelo);
if vPico>=vMaxModelo
    warning('El limite de PWM no permite la velocidad pico; aumentar tiempo o revisar limites.');
end

Nombres = ["Nominal";"Kv -20%, tau +30%, zona muerta +15%";"Kv +20%, tau -20%"];
Factores = [1 1 1;0.8 1.3 1.15;1.2 0.8 1];
SIM_PASO = cell(size(Factores,1),1);
SIM_TRAY = cell(size(Factores,1),1);
metricas = zeros(size(Factores,1),10);
for j = 1:size(Factores,1)
    Ktest=Kv*Factores(j,1); Ttest=tau*Factores(j,2);
    deadTest=PWM_dead*Factores(j,3);
    Gtest = c2d(Ktest/(s*(Ttest*s+1)),Ts,'zoh');
    estable = isstable(feedback(Cd*Gtest,1));
    SIM_PASO{j}=simularPID(t,rPaso,Ktest,Ttest,deadTest,P);
    SIM_TRAY{j}=simularPID(t,rTray,Ktest,Ttest,deadTest,P);
    [mp,mt]=medirRespuesta(t,rPaso,rTray,SIM_PASO{j},SIM_TRAY{j}, ...
        tInicio,TIEMPO_TRAY_S,TOL_FINAL_MM);
    metricas(j,:)=[estable mp mt];
end
Tabla = array2table(metricas,'VariableNames', ...
    {'LinealEstable','Sobreimpulso_pct','T_est_2pct_s','ErrorFinalPaso_mm', ...
    'RMSE_Tray_mm','MaxError_Tray_mm','ErrorFinalTray_mm', ...
    'TiempoFueraTolFinal_s','SaturacionTray_pct','MaxPWMTray_pct'});
Tabla = addvars(Tabla,Nombres,'Before',1,'NewVariableNames','Caso');
disp(Tabla);
if abs(Tabla.ErrorFinalTray_mm(1))>TOL_FINAL_MM
    warning('El caso nominal no cumple la tolerancia final seleccionada.');
end
fprintf('Las metricas anteriores son SIMULADAS; no certifican el equipo real.\n');

%% ============================================================
% 6. GRAFICAS COMPARABLES CON EL SCRIPT DEL MODELO
%% ============================================================
if MOSTRAR_GRAFICAS
    figure('Name','PID - Posicion','NumberTitle','off','Color','w');
    subplot(2,1,1); plot(t,rPaso,'k--','LineWidth',1.5); hold on;
    for j=1:numel(SIM_PASO), plot(t,SIM_PASO{j}.x,'LineWidth',1.3); end
    grid on; ylabel('Posicion [mm]'); title('Escalon: derivada sobre medicion');
    legend(["Referencia";Nombres],'Location','best');
    subplot(2,1,2); plot(t,rTray,'k--','LineWidth',1.5); hold on;
    for j=1:numel(SIM_TRAY), plot(t,SIM_TRAY{j}.x,'LineWidth',1.3); end
    grid on; xlabel('Tiempo [s]'); ylabel('Posicion [mm]');
    title('Trayectoria suave de posicion');

    figure('Name','PID - Mando y error nominal','NumberTitle','off','Color','w');
    subplot(3,1,1); plot(t,SIM_TRAY{1}.pwm,'LineWidth',1.5); grid on;
    ylabel('PWM fisico [%]'); title('Caso nominal: trayectoria');
    subplot(3,1,2); plot(t,rTray-SIM_TRAY{1}.x,'LineWidth',1.5); grid on;
    ylabel('Error [mm]');
    subplot(3,1,3); plot(t,SIM_TRAY{1}.v,'LineWidth',1.5); grid on;
    ylabel('Velocidad [mm/s]'); xlabel('Tiempo [s]');
end

%% ============================================================
% 7. EXPORTAR RESULTADOS POR EJE
%
% Cabecera .h: constantes y tipo de datos; NO es el firmware completo.
% MAT: datos, controlador y curvas para guardar el diseno.
% CSV: tabla de ganancias e indicadores.
% Cambiar EJE_ID en identificacion y repetir para cada conjunto.
%% ============================================================
PID_EJE = P;
PID_EJE.eje = Modelo.eje;
PID_EJE.metodo = METODO;
PID_EJE.unidades_error = 'mm';
PID_EJE.unidades_salida = 'puntos porcentuales de PWM efectivo';
PID_EJE.derivada = 'sobre medicion';
PID_EJE.discretizacion = 'BackwardEuler';
PID_EJE.modelo = Modelo;
PID_EJE.validado_en_motor = false;
Ganancias = table(string(Modelo.eje),Kp,Ki,Kd,Tf,Ts,KI_PASO,A_D,B_D, ...
    PWM_dead,PWM_MAX,U_MAX,q,'VariableNames', ...
    {'Eje','Kp','Ki','Kd','Tf_s','Ts_s','Ki_por_Ts','A_D','B_D', ...
     'PWM_dead_pct','PWM_max_pct','U_max_pct','mm_por_cuenta'});
disp(Ganancias);
if EXPORTAR
    nombre = regexprep(char(Modelo.eje),'[^a-zA-Z0-9_]','_');
    carpeta = fullfile(BASE_DIR,['SalidaPID_' nombre]);
    if ~isfolder(carpeta), mkdir(carpeta); end
    save(fullfile(carpeta,['PID_' nombre '.mat']), ...
        'PID_EJE','Cd','Gx','Gd','Tabla','Ganancias','t','rPaso','rTray', ...
        'SIM_PASO','SIM_TRAY','T_ref','polos_discretos');
    writetable(Ganancias,fullfile(carpeta,['Ganancias_' nombre '.csv']));
    writetable(Tabla,fullfile(carpeta,['Simulaciones_' nombre '.csv']));
    escribirCabecera(fullfile(carpeta,['pid_' nombre '.h']),nombre,P);
    fprintf('\nResultados guardados en: %s\n',carpeta);
end
fprintf('\nLlevar al firmware: ganancias + Ts + filtro + unidades + limites.\n');
fprintf('Mantener estados I, D y posicion anterior independientes por motor.\n');
fprintf('Reiniciar estados al deshabilitar el eje; inicializar xprev con su posicion actual.\n');
fprintf('Validar a baja velocidad, con carga y en ambos sentidos.\n');

%% ============================================================
% FUNCION LOCAL 1. LAZO DISCRETO CON SATURACION Y ANTI-WINDUP
%% ============================================================
function R = simularPID(t,r,Kv,tau,deadReal,P)
    n=numel(t); x=0; v=0; I=0; D=0; xprev=0;
    R.x=zeros(n,1); R.v=zeros(n,1); R.pwm=zeros(n,1);
    R.u=zeros(n,1); R.I=zeros(n,1); R.saturado=false(n,1);
    % Solucion EXACTA de la planta de primer orden para un PWM constante Ts.
    % x[k+1]=x[k]+tau*(1-a)*v[k]+Kv*(Ts-tau*(1-a))*u[k]
    % v[k+1]=a*v[k]+Kv*(1-a)*u[k]
    a=exp(-P.Ts/tau); b=tau*(1-a);
    for k=1:n
        R.x(k)=x; R.v(k)=v;
        if P.cuantizar, y=round(x/P.q)*P.q; else, y=x; end
        e=r(k)-y;
        D=P.A_D*D-P.B_D*(y-xprev);
        Itrial=I+P.KI_PASO*e;
        uTrial=P.Kp*e+Itrial+D;
        empeoraSaturacion=(uTrial>P.U_MAX && e>0) || ...
                          (uTrial<-P.U_MAX && e<0);
        if ~empeoraSaturacion
            I=min(max(Itrial,-P.I_MAX),P.I_MAX);
        end
        uLibre=P.Kp*e+I+D;
        u=min(max(uLibre,-P.U_MAX),P.U_MAX);
        if abs(u)>1e-12
            pwm=sign(u)*(abs(u)+P.PWM_dead);
        else
            pwm=0;
        end
        pwm=min(max(pwm,-P.PWM_MAX),P.PWM_MAX);
        % Planta REAL simulada: zona muerta puede diferir de la compensacion.
        uPlanta=sign(pwm)*max(abs(pwm)-deadReal,0);
        R.u(k)=u; R.pwm(k)=pwm; R.I(k)=I;
        R.saturado(k)=abs(uTrial)>P.U_MAX || abs(uLibre)>P.U_MAX;
        xnext=x+b*v+Kv*(P.Ts-b)*uPlanta;
        vnext=a*v+Kv*(1-a)*uPlanta;
        x=xnext; v=vnext; xprev=y;
        if ~isfinite(x) || abs(x)>1e7
            error('Simulacion divergente: revisar parametros.');
        end
    end
end

%% ============================================================
% FUNCION LOCAL 2. INDICADORES SIN CONFUNDIRLOS CON MEDICIONES
%% ============================================================
function [mp,mt]=medirRespuesta(t,rp,rt,SP,ST,t0,Tmov,tol)
    post=t>=t0; target=rp(end);
    sobre=100*max(0,max(SP.x(post))-target)/abs(target);
    lastBad=find(post & abs(SP.x-target)>0.02*abs(target),1,'last');
    if isempty(lastBad)
        test=0;
    elseif lastBad==numel(t)
        test=NaN; % no establecido dentro del tiempo simulado
    else
        test=t(lastBad+1)-t0;
    end
    % Error final promediado en los ultimos 0.5 s para mostrar oscilaciones aparte.
    cola=t>=t(end)-0.5;
    mp=[sobre,test,mean(target-SP.x(cola))];
    err=rt-ST.x; final=t>=t0+Tmov;
    tiempoFuera=sum(final & abs(err)>tol)*(t(2)-t(1));
    mt=[sqrt(mean(err(post).^2)),max(abs(err(post))),mean(err(cola)), ...
        tiempoFuera,100*mean(ST.saturado(post)),max(abs(ST.pwm))];
end

%% ============================================================
% FUNCION LOCAL 3. CONSTANTES C++ Y CONVENCION DEL ALGORITMO
%% ============================================================
function escribirCabecera(path,nombre,P)
    fid=fopen(path,'w');
    if fid<0, error('No se pudo crear %s.',path); end
    cerrar=onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid,'#pragma once\n\n');
    fprintf(fid,'// CANDIDATAS: verificar en el eje real antes de operar.\n');
    fprintf(fid,'// Error mm; salida %%PWM efectivo; derivada sobre medicion.\n');
    fprintf(fid,'// Integral y derivada Backward Euler. Estados privados por eje.\n');
    fprintf(fid,'// Itrial=I+KiPaso*e; D=Ad*D-Bd*(x-xprev); u=Kp*e+I+D.\n');
    fprintf(fid,'// Aplicar anti-windup y saturacion ANTES de compensar zona muerta.\n');
    fprintf(fid,'// pwm=sign(u)*(abs(u)+dead) si u!=0; pwm=0 si u==0.\n\n');
    fprintf(fid,'#ifndef PID_POSICION_CONFIG_T_DEFINED\n');
    fprintf(fid,'#define PID_POSICION_CONFIG_T_DEFINED\n');
    fprintf(fid,'struct PidPosicionConfig {\n');
    fprintf(fid,'  float kp, ki, kd, tf_s, ts_s;\n');
    fprintf(fid,'  float kiPaso, ad, bd, iMax, pwmDead, pwmMax, uMax, mmPorCuenta;\n');
    fprintf(fid,'};\n#endif\n\n');
    fprintf(fid,'static constexpr PidPosicionConfig PID_%s = {\n',nombre);
    valores=[P.Kp P.Ki P.Kd P.Tf P.Ts P.KI_PASO P.A_D P.B_D ...
        P.I_MAX P.PWM_dead P.PWM_MAX P.U_MAX P.q];
    for j=1:numel(valores)
        if j<numel(valores), coma=','; else, coma=''; end
        fprintf(fid,'  %.10ef%s\n',valores(j),coma);
    end
    fprintf(fid,'};\n');
end
