% REVISION MINIMA: mismo lector TXT y graficas; cargas fisicas independientes.
% No se exportan archivos salvo activar EXPORTAR_MODELO o EXPORTAR_TABLAS.
%% ============================================================
% MODELO DINAMICO Y VALIDACION DEL EJE LINEAL
%
% Driver BLDC + Motor + Husillo + Encoder
%
% El TXT se pega TAL CUAL sale del ESP32:
%
% intento,t_us,pwm,dir,counts,delta_counts,x_mm,v_mm_s,rpm
% El lector convierte us -> ms y calcula aceleracion internamente.
% Conservar las lineas # START, # EVENT y # END del firmware V2.
% Tambien admite el formato antiguo numerado de 10 columnas (t_ms).
%
% El script:
% 1) lee todos los intentos;
% 2) usa desplazamientos reales opcionales para corregir mm/cuenta;
% 3) calcula Vss y tau de cada intento;
% 4) promedia por PWM y direccion;
% 5) identifica Kv y PWM_dead;
% 6) crea Gv(s) y Gx(s);
% 7) compara experimento vs modelo para posicion, velocidad, RPM y aceleracion;
% 8) conserva calculos auxiliares de torque APROXIMADO, no torque medido;
% 9) estima corriente/torque del modelo mecanico equivalente;
% 10) muestra las tablas en MATLAB; exportar CSV es opcional.
% Requiere Control System Toolbox (tf, lsim).
% REVISION: exportacion por eje, controles de calidad y unidades explicitas.
% Gx queda en Workspace; MAT opcional activando EXPORTAR_MODELO.
% Necesita tu TXT real; los valores de respaldo NO prueban una identificacion.
%% ============================================================

clear;
clc;
close all;
BASE_DIR = fileparts(mfilename('fullpath'));

%% DATOS FISICOS ADICIONALES (NO MODIFICAN EL TXT NI SUS COLUMNAS)
% Un motor independiente. Repetir con datos propios para M1 y M2.
% Las curvas x/v se identifican desde TU TXT; estas cargas calculan esfuerzo.
ETAPA_ENSAYO = "sin_clasificar"; % "libre", "compresion", "traslado", "mixto"
INTENTOS_IDENTIFICACION = []; % vacio: comportamiento original; o lista de escalones de MISMA condicion
INTENTOS_COMPRESION = []; % excluir del ajuste Vss/tau si no hay regimen estacionario
EXPORTAR_MODELO = true; % guarda Modelo_Eje1.mat, que necesita DisenoPID_Posicion.m
MOSTRAR_GRAFICA_CARGAS = true;
FISICA.modo = "sujecion";
FISICA.F_resorte_ref_N = 42.88; % aproximada indicada por el usuario: UN resorte
FISICA.compresion_ref_mm = NaN; % compresion correspondiente a 42.88 N; medir
FISICA.precarga_inicial_N = 0; % hipotesis editable; comprobar
FISICA.m_ladrillo_kg = 2.5; % referencia documental; sustituir por pesaje de tu ladrillo
FISICA.m_carro_kg = NaN; % masa movil del mecanismo, SIN el ladrillo
FISICA.mu_equivalente = NaN; % completar resistencia equivalente, no inventar
FISICA.eta_husillo = 0.40; % hipotesis del diseno mecanico, no eficiencia medida
FISICA.eta_reductor = 1;
FISICA.t_contacto_s = NaN; % tiempo relativo al inicio del intento graficado
FISICA.t_traslado_s = NaN; % solo para "mixto": instante medido de cambio de etapa
FISICA.signo_resorte_traslado = 1; % +1 resiste, -1 ayuda, 0 no es carga axial adicional
% +1 es un SUPUESTO de carga resistente mantenida durante traslado.
% Si 42.88 N es solo la fuerza transmitida al ladrillo (no una reaccion externa
% adicional), usar 0: NO sumar dos veces la fuerza de la misma cadena mecanica.
FISICA.J_husillo_kgm2 = 0; % omision provisional; sustituir por dato medido/CAD
FISICA.J_reductor_salida_kgm2 = 0;
FISICA.F_corte_N = 0; % no aplica a este archivo
% Para compresion: se supone placa/ladrillo inmovil mientras avanza el carro.
% Para traslado: compresion aproximadamente mantenida, carro y ladrillo con
% igual aceleracion. Es una aproximacion INDIVIDUAL; no simula otros motores.


%% ============================================================
% 0. VALORES DE RESPALDO
%
% Valores editables que ya tenias. Son estimaciones previas, no mediciones nuevas.
% MODO_MODELO="manual": utiliza exactamente estos tres parametros.
% MODO_MODELO="identificar": estima con TXT; usa respaldo solo si faltan datos.
%% ============================================================

Kv_manual       = 5.9825;    % (mm/s)/%PWM efectivo
tau_manual      = 0.05025;   % s
PWM_dead_manual = 5.546;     % %

EJE_ID = "Eje1";             % Cambiar a Eje2, Eje3... para cada mecanismo
PERMITIR_RESPALDO = false;   % Fallar antes que disenar con datos no identificados
V_MIN_ID = 0.10;             % mm/s: umbral editable segun ruido del encoder
TOL_ESTACIONARIO = 0.10;     % diferencia relativa admisible entre dos colas
R2_MIN = 0.95;              % aviso de calidad; no demuestra validez por si solo
MODO_MODELO = "identificar"; % "identificar" o "manual"
DIR_MODELO = -1;             % 0=promedio original; 1 o -1=solo ese sentido
EXPORTAR_TABLAS = false;    % false: no genera archivos externos
MOSTRAR_CORRIENTE_TORQUE = false; % Conserva calculos originales como opcion
USAR_CALIBRACION_POR_INTENTO = false; % true recupera ajuste por distancia del original
% Para PID es preferible fijar una conversion comun comprobada (LEAD/CPR).
% Separar ensayos de comprobacion cuando ya tengas suficientes repeticiones.
TRIALS_VALIDACION = [];     % Ejemplo [12 27]; se excluyen de la identificacion
assert(any(MODO_MODELO == ["identificar","manual"]), 'MODO_MODELO invalido');
assert(ismember(DIR_MODELO,[-1 0 1]), 'DIR_MODELO debe ser -1, 0 o 1');

%% ============================================================
% 1. DATOS QUE TU MODIFICAS
%% ============================================================

% Archivo TXT copiado directamente del Serial Monitor
SERIAL_FILE = fullfile(BASE_DIR,'pruebas_motor.txt');

% Intento que quieres mostrar en las graficas de validacion
TRIAL_TO_PLOT = 9; % Cambiar a un intento existente en TU TXT real

% ------------------------------------------------------------
% DESPLAZAMIENTO REAL MEDIDO CON CALIBRADOR
%
% Formato:
% [ intento   distancia_real_mm ]
%
% Si no tienes medicion de algun intento, no lo incluyas.
% Tambien puedes dejar:
% REAL_DISPLACEMENT = [];
% ------------------------------------------------------------

REAL_DISPLACEMENT = [1 6.1; 2 5.3; 3 7.5; 4 5.2; 5 10.4; 6 8.7; 7 12.4; 8 10.9; 9 16.9; 10 15.8];
% Introduce tus mediciones nuevas, por ejemplo:
% REAL_DISPLACEMENT = [1 7.90; 2 14.10; 3 21.10];
% Estos numeros son solo el ejemplo antiguo: no activarlos para otros ensayos.

% ------------------------------------------------------------
% CORRIENTE PROMEDIO MEDIDA MANUALMENTE
%
% Formato:
% [ intento   corriente_A ]
%
% Si es corriente del bus de 24 V antes del driver, el torque
% calculado con T = Kt*I debe reportarse como ESTIMACION.
%
% Puedes dejar:
% CURRENT_AVERAGE = [];
% ------------------------------------------------------------

CURRENT_AVERAGE = [1 0.06; 2 0.07; 3 0.06; 4 0.07; 5 0.06; 6 0.08; 7 0.06; 8 0.06; 9 0.07; 10 0.09];
% CURRENT_AVERAGE = [1 0.06; 2 0.07; 3 0.06]; % Ejemplo, reemplazar por mediciones
% Una corriente de bus del driver NO es directamente la corriente de motor.
% No utilizar Kt*I_bus para afirmar torque medido.

% ------------------------------------------------------------
% CORRIENTE CONTRA TIEMPO (OPCIONAL)
%
% Si posteriormente mides corriente en varios instantes:
%
% [ intento   t_ms   corriente_A ]
%
% Si esta vacio, se utiliza CURRENT_AVERAGE.
%
% IMPORTANTE:
% Esta variable puede quedar completamente vacia sin producir error.
% ------------------------------------------------------------

CURRENT_TIME_SERIES = [];

%% ============================================================
% 2. PARAMETROS MECANICOS
%% ============================================================

CPR  = 1000;      % CUENTAS DECODIFICADAS por vuelta del eje del encoder
% No confundir PPR de ficha con cuentas x1/x2/x4 del firmware.
% Ejemplo: 1000 PPR leidos x4 -> CPR=4000, si esa es tu decodificacion.
LEAD = 8;         % mm/vuelta del husillo
RELACION_ENCODER_HUSILLO = 1; % vueltas encoder / vuelta husillo
% Encoder en motor y reductor 10:1 -> relacion=10; en husillo -> relacion=1.
assert(CPR>0 && LEAD>0 && RELACION_ENCODER_HUSILLO>0, ...
    'Verificar CPR, avance del husillo y relacion mecanica.');
MM_PER_COUNT_NOMINAL = LEAD/(CPR*RELACION_ENCODER_HUSILLO);
COUNTS_PER_MM        = 1/MM_PER_COUNT_NOMINAL;
% El firmware debe usar la MISMA conversion. Su velocidad se escala abajo.
MM_PER_COUNT_FIRMWARE = 8/1000; % valor usado al generar ESTE TXT; verificar
assert(MM_PER_COUNT_FIRMWARE>0, 'Conversion del firmware invalida');
LEAD_M               = LEAD/1000;

%% ============================================================
% 3. DATOS EDITABLES DE FICHA TECNICA DEL MOTOR (VERIFICAR CON TU MOTOR)
%% ============================================================

V_NOM          = 24;       % V
P_NOM          = 70;       % W
RPM_NOM        = 3000;     % rpm
RPM_MAX        = 4000;     % rpm
T_NOM          = 0.22;     % N*m
I_NOM          = 4.8;      % A
KE_V_KRPM      = 3.4;      % V/krpm
J_ROTOR_KGMM2  = 11.9;     % kg*mm^2

% Constantes derivadas (aproximaciones; no se identifican desde el encoder)
% Kt=Tnom/Inom es una aproximacion, no reemplaza la constante de ficha tecnica.
% Estas constantes NO intervienen en la identificacion empirica de Gx.
Kt = T_NOM/I_NOM;                         % N*m/A
omega_1000rpm = 1000*2*pi/60;            % rad/s
Ke = KE_V_KRPM/omega_1000rpm;            % V/(rad/s)
J_rotor = J_ROTOR_KGMM2*1e-6;            % kg*m^2

%% ============================================================
% 4. PARAMETROS DEL MODELO MECANICO PARA ESTIMAR I Y T
%
% T = J*alpha + B*omega + T_load
% I = T/Kt
%
% J_EQ debe ser la inercia equivalente referida al eje.
% Por ahora se usa solo la inercia del rotor.
%% ============================================================

J_EQ = J_rotor;

% Estimacion inicial de friccion viscosa:
% B = Kt*I0/omega0
I0_NO_LOAD   = 0.06;     % A
RPM0_NO_LOAD = 195;      % rpm

if RPM0_NO_LOAD > 0
    omega0 = RPM0_NO_LOAD*2*pi/60;
    B_VISCOUS = Kt*I0_NO_LOAD/omega0;
else
    B_VISCOUS = 0;
end

% Torque externo aplicado durante la caracterizacion
T_LOAD = 0;              % N*m

%% ============================================================
% 5. FILTRADO Y UMBRALES
%% ============================================================

VEL_FILTER = 5;
ACC_FILTER = 7;

PWM_ACTIVE_THRESHOLD = 0.001;
STEADY_FRACTION = 0.30;

%% ============================================================
% 6. LEER TXT DEL ESP32
%% ============================================================

[DATA, META] = readSerialFile(SERIAL_FILE);

if isempty(DATA)
    error('No se encontraron filas numericas validas en "%s".', SERIAL_FILE);
end

fprintf('\n========================================\n');
fprintf('ARCHIVO CARGADO\n');
fprintf('========================================\n');
fprintf('Archivo        : %s\n', SERIAL_FILE);
fprintf('Filas leidas  : %d\n', size(DATA,1));
fprintf('Intentos       : %d\n', numel(unique(DATA(:,1))));

%% ============================================================
% COLUMNAS INTERNAS DATA DESPUES DE ADAPTAR EL TXT
%
% 1  intento
% 2  t_ms
% 3  pwm
% 4  dir
% 5  counts
% 6  delta_counts
% 7  x_mm calculado en ESP32
% 8  v_mm_s calculado en ESP32
% 9  rpm calculado en ESP32
% 10 a_mm_s2 calculado en MATLAB para entrada V2
%% ============================================================

%% ============================================================
% 7. ANALIZAR CADA INTENTO
%% ============================================================

TRIALS = unique(DATA(:,1));
RESULTS = [];

for n = 1:numel(TRIALS)

    trial = TRIALS(n);
    if ismember(trial,TRIALS_VALIDACION) || ismember(trial,INTENTOS_COMPRESION) || ...
            (~isempty(INTENTOS_IDENTIFICACION) && ~ismember(trial,INTENTOS_IDENTIFICACION))
        continue;
    end
    D = DATA(DATA(:,1)==trial,:);

    % Eliminar filas no finitas, ordenar y quitar tiempos repetidos
    [D, ok] = cleanTrialMatrix(D);

    if ~ok
        warning('Intento %d ignorado: no tiene suficientes muestras validas.', trial);
        continue;
    end

    t        = D(:,2)/1000;
    pwm      = D(:,3);
    dir      = D(:,4);
    delta    = D(:,6);
    vArduino = D(:,8);

    % Hacer que cada ensayo empiece exactamente en t = 0
    t = t - t(1);

    finalCounts = abs(delta(end)-delta(1));

    if finalCounts <= 0
        warning('Intento %d ignorado: desplazamiento final igual a cero.', trial);
        continue;
    end

    % --------------------------------------------------------
    % Calibracion cuentas -> mm usando distancia fisica real
    % --------------------------------------------------------

    idxReal = [];

    if ~isempty(REAL_DISPLACEMENT) && size(REAL_DISPLACEMENT,2) >= 2
        idxReal = find(REAL_DISPLACEMENT(:,1)==trial,1);
    end

    if ~isempty(idxReal) && USAR_CALIBRACION_POR_INTENTO
        realDistance = REAL_DISPLACEMENT(idxReal,2);
        mmPerCountTrial = realDistance/finalCounts;
    else
        realDistance = finalCounts*MM_PER_COUNT_NOMINAL;
        mmPerCountTrial = MM_PER_COUNT_NOMINAL;
    end

    calibrationFactor = mmPerCountTrial/MM_PER_COUNT_FIRMWARE;

    % Trabajar en magnitud positiva para caracterizacion
    movementSign = sign(delta(end)-delta(1));
    if movementSign == 0
        movementSign = 1;
    end

    % Posicion y velocidad experimentales
    xExp = (delta-delta(1))*mmPerCountTrial*movementSign;

    vExp = vArduino*calibrationFactor*movementSign;
    % Sin filtro previo para no deformar el cruce usado para tau.
    % El suavizado se conserva despues, para mostrar las graficas.

    % --------------------------------------------------------
    % Intervalo con PWM activo
    % --------------------------------------------------------

    active = find(abs(pwm)>PWM_ACTIVE_THRESHOLD);

    if isempty(active)
        warning('Intento %d ignorado: no tiene PWM activo.', trial);
        continue;
    end

    PWMlevel = median(abs(pwm(active)));
    DIRlevel = round(median(dir(active)));
    if DIR_MODELO ~= 0 && DIRlevel ~= DIR_MODELO, continue; end

    % Un intento util debe contener UN escalon de PWM y UN sentido.
    if any(diff(active) ~= 1) || ...
            max(abs(abs(pwm(active))-PWMlevel)) > max(0.05,0.01*PWMlevel) || ...
            any(dir(active) ~= DIRlevel)
        warning('Intento %d: PWM variable, varios pulsos o cambio de sentido; omitido.',trial);
        continue;
    end

    % --------------------------------------------------------
    % Velocidad estacionaria
    % Se toma el ultimo porcentaje del tramo PWM ON
    % --------------------------------------------------------

    Nss = max(3,round(STEADY_FRACTION*numel(active)));
    Nss = min(Nss,numel(active));

    idxSS = active(end-Nss+1:end);
    Vss = median(vExp(idxSS),'omitnan');
    if ~isfinite(Vss) || Vss < V_MIN_ID
        warning('Intento %d: sin movimiento fiable; no usar en ajuste de zona activa.',trial);
        continue;
    end
    if Nss < 6
        warning('Intento %d: menos de 6 muestras en cola; aumentar duracion.',trial);
        continue;
    end
    mitad = floor(Nss/2);
    cola1 = median(vExp(idxSS(1:mitad)));
    cola2 = median(vExp(idxSS(mitad+1:end)));
    if abs(cola2-cola1) > TOL_ESTACIONARIO*max(Vss,V_MIN_ID)
        warning('Intento %d: cola no estacionaria; omitido.',trial);
        continue;
    end
    % La prueba de las dos colas es un filtro basico, no certifica estacionariedad.
    % Confirmar graficamente, duracion suficiente y repeticiones con carga.


    % --------------------------------------------------------
    % Tau experimental:
    % v(tau) = 0.632*Vss
    % --------------------------------------------------------

    tauTrial = NaN;

    if isfinite(Vss) && Vss > 0

        targetTau = (1-exp(-1))*Vss;
        % Formula valida para escalon desde reposo y retardo despreciable.
        pre = find(t < t(active(1)));
        if isempty(pre)
            warning('Intento %d: sin muestras previas al escalon; revisar reposo y tau.',trial);
        elseif abs(median(vExp(pre(max(1,end-4):end)))) > 0.10*Vss
            warning('Intento %d: no parte del reposo; omitido.',trial);
            continue;
        end
        activeVelocity = vExp(active);

        crossIndex = find(activeVelocity>=targetTau,1,'first');

        if ~isempty(crossIndex)

            tStartPWM = t(active(1));
            im = find([META.trial] == trial,1);
            if ~isempty(im) && isfinite(META(im).tOn)
                tStartPWM = META(im).tOn - D(1,2)/1000;
            end

            if crossIndex == 1
                tCross = t(active(1));
            else
                k1 = active(crossIndex-1);
                k2 = active(crossIndex);

                t1 = t(k1);
                t2 = t(k2);
                v1 = vExp(k1);
                v2 = vExp(k2);

                if abs(v2-v1) > 1e-12
                    tCross = t1 + (targetTau-v1)*(t2-t1)/(v2-v1);
                else
                    tCross = t2;
                end
            end

            tauTrial = tCross - tStartPWM;

            if tauTrial <= 0 || ~isfinite(tauTrial)
                tauTrial = NaN;
            end
        end
    end

    if isfinite(tauTrial)
        dtID = median(diff(t));
        if dtID > tauTrial/5
            warning('Intento %d: pocas muestras por tau; mejorar muestreo de identificacion.',trial);
        end
        if numel(active)*dtID < 5*tauTrial
            warning('Intento %d: pulso menor a 5*tau; Vss y tau pueden estar sesgados.',trial);
        end
    end

    RESULTS = [
        RESULTS;
        trial, DIRlevel, PWMlevel, Vss, tauTrial, ...
        realDistance, mmPerCountTrial
    ];
end

if isempty(RESULTS)
    warning('Sin ensayos ID utilizables; revisar direccion, archivo y criterios de calidad.');
    RESULTS = zeros(0,7);
end

%% ============================================================
% 8. TABLA POR INTENTO
%% ============================================================

ResultTable = array2table(RESULTS, ...
    'VariableNames',{ ...
    'Intento','DIR','PWM','Vss_mm_s','Tau_s', ...
    'DistanciaUsada_mm','mm_por_cuenta'});

fprintf('\n========================================\n');
fprintf('RESULTADOS POR INTENTO\n');
fprintf('========================================\n');
disp(ResultTable);

%% ============================================================
% 9. PROMEDIOS POR PWM Y DIRECCION
%% ============================================================

pairs = unique(RESULTS(:,2:3),'rows');
AVERAGES = zeros(0,6);

for k = 1:size(pairs,1)

    d = pairs(k,1);
    p = pairs(k,2);

    idx = RESULTS(:,2)==d & abs(RESULTS(:,3)-p)<1e-9;
    subset = RESULTS(idx,:);

    AVERAGES = [
        AVERAGES;
        d, p, ...
        mean(subset(:,4),'omitnan'), ...
        std(subset(:,4),'omitnan'), ...
        mean(subset(:,5),'omitnan'), ...
        std(subset(:,5),'omitnan')
    ];
end

AverageTable = array2table(AVERAGES, ...
    'VariableNames',{ ...
    'DIR','PWM','Vss_promedio','Vss_std', ...
    'Tau_promedio','Tau_std'});

fprintf('\n========================================\n');
fprintf('PROMEDIOS POR PWM Y DIRECCION\n');
fprintf('========================================\n');
disp(AverageTable);

%% ============================================================
% 10. IDENTIFICAR Kv Y PWM DEAD
%
% v = Kv*PWM + b
% PWM_dead = -b/Kv
%% ============================================================

directions = unique(AVERAGES(:,1));

KvValues = [];
DeadValues = [];
DIRECTION_MODELS = zeros(0,6);

for k = 1:numel(directions)

    d = directions(k);

    idx = AVERAGES(:,1)==d;

    PWMdir = AVERAGES(idx,2);
    Vdir   = AVERAGES(idx,3);

    valid = isfinite(PWMdir) & isfinite(Vdir) & Vdir>=V_MIN_ID;

    PWMdir = PWMdir(valid);
    Vdir   = Vdir(valid);

    if numel(unique(PWMdir)) >= 2

        P = polyfit(PWMdir,Vdir,1);

        KvDir = P(1);

        if isfinite(KvDir) && KvDir > 0
            DeadDir = -P(2)/P(1);
            if ~isfinite(DeadDir) || DeadDir < 0 || DeadDir >= min(PWMdir)
                warning('DIR %d: zona muerta fuera de rango; se omite este ajuste.',d);
                continue;
            end

            residual = Vdir-polyval(P,PWMdir);
            SST = sum((Vdir-mean(Vdir)).^2);
            R2 = 1-sum(residual.^2)/max(SST,eps);
            tauD = RESULTS(RESULTS(:,2)==d,5);
            tauD = tauD(isfinite(tauD) & tauD>0);
            if isempty(tauD), tauD = NaN; else, tauD=mean(tauD); end
            DIRECTION_MODELS(end+1,:) = [d KvDir tauD DeadDir R2 numel(PWMdir)];
            fprintf('R2 ajuste = %.5f | niveles PWM = %d\n',R2,numel(PWMdir));
            if numel(PWMdir)<3 || R2<R2_MIN
                warning('DIR %d: usar al menos 3 niveles activos y revisar residuos del ajuste.',d);
            end
            KvValues   = [KvValues; KvDir];
            DeadValues = [DeadValues; DeadDir];

            fprintf('\nDIR %d\n',d);
            fprintf('Kv identificado       = %.6f (mm/s)/%%PWM\n',KvDir);
            fprintf('PWM dead identificado = %.6f %%\n',DeadDir);
        end
    end
end

validKv = KvValues(isfinite(KvValues) & KvValues>0);
validDead = DeadValues(isfinite(DeadValues));

if isempty(validKv)
    fprintf('\nNo hay suficientes niveles de PWM para identificar Kv.\n');
    fprintf('Se usa Kv_manual = %.6f\n',Kv_manual);
    if MODO_MODELO=="identificar" && ~PERMITIR_RESPALDO
        error('Kv no identificado. Revisar ensayos o elegir manual EXPLICITAMENTE.');
    end
    Kv = Kv_manual; fuenteKv = "respaldo manual";
else
    Kv = mean(validKv); fuenteKv = "estimado del TXT";
end

if isempty(validDead)
    fprintf('No hay suficientes datos para identificar PWM_dead.\n');
    fprintf('Se usa PWM_dead_manual = %.6f %%\n',PWM_dead_manual);
    if MODO_MODELO=="identificar" && ~PERMITIR_RESPALDO
        error('Zona muerta no identificada. Revisar ensayos.');
    end
    PWM_dead = PWM_dead_manual; fuenteDead = "respaldo manual";
else
    PWM_dead = mean(validDead); fuenteDead = "estimado del TXT";
end

%% ============================================================
% 11. IDENTIFICAR TAU
%% ============================================================

validTau = RESULTS(:,5);
validTau = validTau(isfinite(validTau) & validTau>0);

if isempty(validTau)
    fprintf('No fue posible identificar tau experimentalmente.\n');
    fprintf('Se usa tau_manual = %.6f s\n',tau_manual);
    if MODO_MODELO=="identificar" && ~PERMITIR_RESPALDO
        error('Tau no identificada. Revisar resolucion temporal y escalon desde reposo.');
    end
    tau = tau_manual; fuenteTau = "respaldo manual";
else
    tau = mean(validTau); fuenteTau = "estimado del TXT";
end

% Eleccion explicita del usuario, conservando los parametros editables.
if MODO_MODELO == "manual"
    Kv = Kv_manual; tau = tau_manual; PWM_dead = PWM_dead_manual;
    fuenteKv = "manual elegido"; fuenteTau = "manual elegido"; fuenteDead = "manual elegido";
end
fprintf('\nORIGEN DE PARAMETROS UTILIZADOS\n');
fprintf('Kv: %s | tau: %s | zona muerta: %s\n',fuenteKv,fuenteTau,fuenteDead);
if DIR_MODELO == 0
    fprintf('Modelo promedio de los sentidos disponibles, como en el original.\n');
else
    fprintf('Modelo del sentido DIR=%d.\n',DIR_MODELO);
end

%% ============================================================
% 12. VERIFICAR CONSTANTES ANTES DE CREAR G(s)
%% ============================================================

fprintf('\n========================================\n');
fprintf('MODELO UTILIZADO PARA COMPARACION Y FUTURO PID\n');
fprintf('========================================\n');
fprintf('Kv       = %.8f (mm/s)/%%PWM\n',Kv);
fprintf('Tau      = %.8f s\n',tau);
fprintf('PWM dead = %.8f %%\n',PWM_dead);

if ~isfinite(Kv) || Kv<=0
    error('Kv no es valido.');
end

if ~isfinite(tau) || tau<=0
    error('Tau no es valido.');
end

if ~isfinite(PWM_dead) || PWM_dead < 0 || PWM_dead >= 100
    error('PWM_dead no es valido.');
end

%% ============================================================
% 13. FUNCIONES DE TRANSFERENCIA
%% ============================================================

s = tf('s');

Gv = Kv/(tau*s + 1);
Gx = Gv/s;
% Para PID: entrada = puntos porcentuales de PWM EFECTIVO; salida = mm.
% La zona muerta y la saturacion deben tratarse aparte al implementar el control.
% No se calcula ni aplica un PID en este script.

fprintf('\nGv(s):\n');
Gv

fprintf('\nGx(s):\n');
Gx

fprintf('\nPolos Gv:\n');
disp(pole(Gv));

fprintf('\nPolos Gx:\n');
disp(pole(Gx));

%% ============================================================
% 13B. EXPORTAR MODELO PARA DISENO PID (ANTES DE LAS GRAFICAS)
% Una ejecucion por eje. No reutilizar un modelo para otros motores sin medir.
%% ============================================================
Modelo = struct();
Modelo.eje = EJE_ID;
Modelo.etapa_declarada = ETAPA_ENSAYO;
Modelo.datos_fisicos = FISICA;
Modelo.Kv = Kv; Modelo.tau = tau; Modelo.PWM_dead = PWM_dead;
Modelo.mm_por_cuenta = MM_PER_COUNT_NOMINAL;
Modelo.CPR = CPR; Modelo.LEAD_mm = LEAD;
Modelo.relacion_encoder_husillo = RELACION_ENCODER_HUSILLO;
Modelo.DIR_MODELO = DIR_MODELO;
Modelo.fuente = [fuenteKv fuenteTau fuenteDead];
Modelo.archivo = SERIAL_FILE;
Modelo.fecha = datestr(now,30);
Modelo.tiene_validacion_reservada = ismember(TRIAL_TO_PLOT,TRIALS_VALIDACION);
% La existencia de un ensayo reservado NO significa que haya pasado validacion.
Modelo.validacion_aprobada = false; % revisar RMSE, graficas y criterio experimental
Modelo.por_direccion = array2table(DIRECTION_MODELS, ...
    'VariableNames',{'DIR','Kv','tau','PWM_dead','R2','niveles_PWM'});
Modelo.Gv = Gv; Modelo.Gx = Gx;
if USAR_CALIBRACION_POR_INTENTO
    error('Para exportar un modelo PID fijar una conversion COMUN, no por intento.');
end
modelPath = fullfile(fileparts(mfilename('fullpath')), "Modelo_"+EJE_ID+".mat");
if EXPORTAR_MODELO
    save(modelPath,'Modelo');
    fprintf('Modelo guardado: %s\n',modelPath);
end

%% ============================================================
% 14. SELECCIONAR INTENTO PARA COMPARACION
%% ============================================================

D = DATA(DATA(:,1)==TRIAL_TO_PLOT,:);

if isempty(D)
    error('El intento %d no existe en el TXT.',TRIAL_TO_PLOT);
end

[D, ok] = cleanTrialMatrix(D);

if ~ok
    error('El intento seleccionado no tiene suficientes datos validos.');
end

tExp       = D(:,2)/1000;
pwmExp     = D(:,3);
deltaExp   = D(:,6);
vArduino   = D(:,8);
rpmArduino = D(:,9);
aArduino   = D(:,10);

tExp = tExp - tExp(1);
if DIR_MODELO ~= 0 && round(median(D(:,4))) ~= DIR_MODELO
    warning('El ensayo graficado tiene otro sentido que el modelo seleccionado.');
end
if MODO_MODELO == "identificar" && ~ismember(TRIAL_TO_PLOT,TRIALS_VALIDACION)
    fprintf('Comparacion con ensayo no reservado: NO es validacion independiente.\n');
end

%% ============================================================
% 15. CALIBRACION DEL INTENTO SELECCIONADO
%% ============================================================

finalCounts = abs(deltaExp(end)-deltaExp(1));

if finalCounts <= 0
    warning('Ensayo seleccionado sin movimiento; comparacion de reposo/zona muerta.');
end

idxReal = [];

if ~isempty(REAL_DISPLACEMENT) && size(REAL_DISPLACEMENT,2) >= 2
    idxReal = find(REAL_DISPLACEMENT(:,1)==TRIAL_TO_PLOT,1);
end

if ~isempty(idxReal) && USAR_CALIBRACION_POR_INTENTO && finalCounts > 0
    distanceReal = REAL_DISPLACEMENT(idxReal,2);
    mmPerCountReal = distanceReal/finalCounts;
else
    distanceReal = finalCounts*MM_PER_COUNT_NOMINAL;
    mmPerCountReal = MM_PER_COUNT_NOMINAL;
end

factorReal = mmPerCountReal/MM_PER_COUNT_FIRMWARE;

motionSign = sign(deltaExp(end)-deltaExp(1));

if motionSign == 0
    motionSign = 1;
end

%% ============================================================
% 16. SEÑALES EXPERIMENTALES
%% ============================================================

xReal = (deltaExp-deltaExp(1))*mmPerCountReal*motionSign;

vReal = vArduino*factorReal*motionSign;
vReal = smoothdata(vReal,'movmean',VEL_FILTER);

% RPM reconstruidas de la velocidad lineal y de la transmision comprobada.
% Se refieren al eje del encoder (motor si el encoder esta en el motor).
rpmReal = vArduino*factorReal*motionSign*60/LEAD*RELACION_ENCODER_HUSILLO;
rpmReal = smoothdata(rpmReal,'movmean',VEL_FILTER);

% Derivar velocidad ya suavizada. La aceleracion es exploratoria.
aReal = gradient(vReal,tExp);
aReal = smoothdata(aReal,'movmean',ACC_FILTER);

PWMphysicalExp = abs(pwmExp);

%% ============================================================
% 17. CREAR TIEMPO UNIFORME PARA LSIM
%
% El TXT puede tener jitter real de muestreo.
% NO se modifica el TXT.
% Solo se crea una malla uniforme interna para simular.
%% ============================================================

dtExp = diff(tExp);
dtExp = dtExp(isfinite(dtExp) & dtExp>0);

if isempty(dtExp)
    error('No se pudo determinar el periodo de muestreo.');
end

Ts_model = min(median(dtExp),tau/20);
% Malla de simulacion mas fina: no agrega informacion al experimento.

if ~isfinite(Ts_model) || Ts_model<=0
    Ts_model = 0.010;
end

nSteps = ceil(tExp(end)/Ts_model);
tModel = (0:nSteps)'*Ts_model;

% PWM fisico llevado a la malla uniforme
pwmModel = interp1( ...
    tExp, ...
    PWMphysicalExp, ...
    tModel, ...
    'previous', ...
    'extrap');

% V2: usar instante ON/OFF exacto si existe; reduce error por jitter del registro.
im = find([META.trial]==TRIAL_TO_PLOT,1);
if ~isempty(im) && isfinite(META(im).tOn) && isfinite(META(im).tOff)
    tOnRel = META(im).tOn-D(1,2)/1000;
    tOffRel = META(im).tOff-D(1,2)/1000;
    levelOn = median(PWMphysicalExp(PWMphysicalExp>PWM_ACTIVE_THRESHOLD));
    if isfinite(levelOn)
        pwmModel = levelOn*double(tModel>=tOnRel & tModel<tOffRel);
    end
end
% PWM efectivo del modelo
pwmEffectiveModel = max(pwmModel-PWM_dead,0);

%% ============================================================
% 18. SIMULAR MODELO
%% ============================================================

% Simular ambos estados juntos, respetando posicion y velocidad iniciales.
plantaXV = ss([0 1;0 -1/tau],[0;Kv/tau],eye(2),zeros(2,1));
vInitial = vArduino(1)*factorReal*motionSign;
xvModel = lsim(plantaXV,pwmEffectiveModel,tModel,[xReal(1);vInitial],'zoh');
xModelUniform = xvModel(:,1);
vModelUniform = xvModel(:,2);

rpmModelUniform = vModelUniform*60/LEAD*RELACION_ENCODER_HUSILLO;

aModelUniform = gradient(vModelUniform,tModel);

% Para el calculo auxiliar J/Kt se supone encoder en el motor.
% Si esta en otro eje, introducir por separado su relacion respecto al motor.
omegaModelUniform = rpmModelUniform*2*pi/60;
alphaModelUniform = gradient(omegaModelUniform,tModel);

% Regresar el modelo a los tiempos experimentales
vModel = interp1(tModel,vModelUniform,tExp,'linear','extrap');
xModel = interp1(tModel,xModelUniform,tExp,'linear','extrap');

rpmModel = interp1(tModel,rpmModelUniform,tExp,'linear','extrap');
aModel = interp1(tModel,aModelUniform,tExp,'linear','extrap');

omegaModel = interp1(tModel,omegaModelUniform,tExp,'linear','extrap');
alphaModel = interp1(tModel,alphaModelUniform,tExp,'linear','extrap');

%% ============================================================
% 19. CORRIENTE Y TORQUE DEL MODELO MECANICO EQUIVALENTE
%
% T = J*alpha + B*omega + T_load
% I = T/Kt
%
% Este calculo es una ESTIMACION mecanica equivalente.
%% ============================================================

Cargas = cargasFisicas(tExp,xModel,vModel,aModel,FISICA, ...
    ETAPA_ENSAYO,LEAD,RELACION_ENCODER_HUSILLO);
% La carga modifica el esfuerzo estimado. NO se resta otra vez de Gx:
% la respuesta identificada ya incorpora la carga presente durante el ensayo.
torqueModelSigned = J_EQ.*alphaModel + B_VISCOUS.*omegaModel + T_LOAD + Cargas.T_carga_Nm;

currentModel = abs(torqueModelSigned)/Kt;

% Corriente equivalente necesaria, no corriente de bus ni consumo medido.
% Sin PWM no se conoce como el driver sostiene/frena: no imponer consumo cero.
currentModel(PWMphysicalExp<=PWM_ACTIVE_THRESHOLD) = NaN;

torqueModel = abs(torqueModelSigned);

%% ============================================================
% 20. CORRIENTE EXPERIMENTAL MANUAL
%
% ESTA SECCION CORRIGE EL ERROR:
% "Index in position 2 exceeds array bounds"
%
% CURRENT_TIME_SERIES puede estar vacio.
%% ============================================================

currentReal = NaN(size(tExp));

% ------------------------------------------------------------
% Opcion A: corriente medida contra tiempo
% ------------------------------------------------------------

if ~isempty(CURRENT_TIME_SERIES) && size(CURRENT_TIME_SERIES,2) >= 3

    idxCurrentSeries = CURRENT_TIME_SERIES(:,1)==TRIAL_TO_PLOT;

    if any(idxCurrentSeries)

        currentTime  = CURRENT_TIME_SERIES(idxCurrentSeries,2)/1000;
        currentValue = CURRENT_TIME_SERIES(idxCurrentSeries,3);

        validCurrentSeries = ...
            isfinite(currentTime) & isfinite(currentValue);

        currentTime  = currentTime(validCurrentSeries);
        currentValue = currentValue(validCurrentSeries);

        if numel(currentTime) >= 2

            [currentTime, ia] = unique(currentTime,'stable');
            currentValue = currentValue(ia);

            currentReal = interp1( ...
                currentTime, ...
                currentValue, ...
                tExp, ...
                'linear', ...
                NaN);

        elseif numel(currentTime) == 1

            activeCurrent = PWMphysicalExp>PWM_ACTIVE_THRESHOLD;
            currentReal(activeCurrent) = currentValue(1);
        end
    end
end

% ------------------------------------------------------------
% Opcion B: corriente promedio por intento
% Solo se usa si no se obtuvo curva contra tiempo.
% ------------------------------------------------------------

if ~any(isfinite(currentReal))

    if ~isempty(CURRENT_AVERAGE) && size(CURRENT_AVERAGE,2) >= 2

        idxCurrentAverage = find( ...
            CURRENT_AVERAGE(:,1)==TRIAL_TO_PLOT, ...
            1);

        if ~isempty(idxCurrentAverage)

            currentAverageValue = CURRENT_AVERAGE(idxCurrentAverage,2);

            activeCurrent = PWMphysicalExp>PWM_ACTIVE_THRESHOLD;

            currentReal(activeCurrent) = currentAverageValue;
        end
    end
end

%% ============================================================
% 21. TORQUE ESTIMADO DESDE CORRIENTE EXPERIMENTAL
%% ============================================================

torqueReal = Kt*currentReal;

%% ============================================================
% 22. ERRORES DE VALIDACION
%% ============================================================

RMSE_position = sqrt(mean((xReal-xModel).^2,'omitnan'));
RMSE_velocity = sqrt(mean((vReal-vModel).^2,'omitnan'));
RMSE_rpm = sqrt(mean((rpmReal-rpmModel).^2,'omitnan'));
RMSE_acceleration = sqrt(mean((aReal-aModel).^2,'omitnan'));

validI = isfinite(currentReal) & isfinite(currentModel);

if any(validI)
    RMSE_current = sqrt(mean((currentReal(validI)-currentModel(validI)).^2));
else
    RMSE_current = NaN;
end

validT = isfinite(torqueReal) & isfinite(torqueModel);

if any(validT)
    RMSE_torque = sqrt(mean((torqueReal(validT)-torqueModel(validT)).^2));
else
    RMSE_torque = NaN;
end

fprintf('\n========================================\n');
fprintf('VALIDACION - INTENTO %d\n',TRIAL_TO_PLOT);
fprintf('========================================\n');
fprintf('Paso simulacion    = %.6f s\n',Ts_model);
fprintf('Ts registro mediano= %.6f s\n',median(dtExp));
fprintf('Distancia usada    = %.6f mm (encoder o calibracion habilitada)\n',distanceReal);
fprintf('mm/cuenta usado    = %.9f mm/cuenta\n',mmPerCountReal);
fprintf('RMSE posicion      = %.6f mm\n',RMSE_position);
fprintf('RMSE velocidad     = %.6f mm/s\n',RMSE_velocity);
fprintf('RMSE RPM           = %.6f rpm\n',RMSE_rpm);
fprintf('RMSE aceleracion   = %.6f mm/s^2\n',RMSE_acceleration);

if MOSTRAR_CORRIENTE_TORQUE && isfinite(RMSE_current)
    fprintf('RMSE corriente     = %.6f A\n',RMSE_current);
end

if MOSTRAR_CORRIENTE_TORQUE && isfinite(RMSE_torque)
    fprintf('RMSE torque        = %.6f N*m\n',RMSE_torque);
end

%% ============================================================
% 23. GRAFICA PWM
%% ============================================================

figure('Name','PWM','NumberTitle','off');

plot(tExp,PWMphysicalExp,'LineWidth',1.8);
hold on;

PWM_effective_exp = max(PWMphysicalExp-PWM_dead,0);

plot(tExp,PWM_effective_exp,'--','LineWidth',1.8);

grid on;
box on;
set(gca,'XMinorGrid','off','YMinorGrid','off');
set(gca,'FontSize',11,'LineWidth',1);

xlabel('Tiempo, t [s]');
ylabel('PWM [%]');
title('Entrada PWM');

legend('PWM fisico','PWM efectivo','Location','best');

%% ============================================================
% 24. POSICION REAL VS MODELO
%% ============================================================

figure('Name','Posicion','NumberTitle','off');

plot(tExp,xReal,'LineWidth',1.8);
hold on;
plot(tExp,xModel,'--','LineWidth',1.8);

grid on;
box on;
set(gca,'XMinorGrid','off','YMinorGrid','off');
set(gca,'FontSize',11,'LineWidth',1);

xlabel('Tiempo, t [s]');
ylabel('Posicion lineal, x [mm]');
title('Comparacion de posicion: encoder vs modelo');

legend('Experimental','Modelo','Location','best');

%% ============================================================
% 25. VELOCIDAD REAL VS MODELO
%% ============================================================

figure('Name','Velocidad','NumberTitle','off');

plot(tExp,vReal,'LineWidth',1.8);
hold on;
plot(tExp,vModel,'--','LineWidth',1.8);

grid on;
box on;
set(gca,'XMinorGrid','off','YMinorGrid','off');
set(gca,'FontSize',11,'LineWidth',1);

xlabel('Tiempo, t [s]');
ylabel('Velocidad lineal, v [mm/s]');
title('Comparacion de velocidad: encoder vs modelo');

legend('Experimental','Modelo','Location','best');

%% ============================================================
% 26. RPM REAL VS MODELO
%% ============================================================

figure('Name','RPM','NumberTitle','off');

plot(tExp,rpmReal,'LineWidth',1.8);
hold on;
plot(tExp,rpmModel,'--','LineWidth',1.8);

grid on;
box on;
set(gca,'XMinorGrid','off','YMinorGrid','off');
set(gca,'FontSize',11,'LineWidth',1);

xlabel('Tiempo, t [s]');
ylabel('Velocidad angular [rpm]');
title('Validacion de velocidad del motor');

legend('Experimental','Modelo','Location','best');

%% ============================================================
% 27. ACELERACION REAL VS MODELO
%% ============================================================

figure('Name','Aceleracion','NumberTitle','off');

plot(tExp,aReal,'LineWidth',1.8);
hold on;
plot(tExp,aModel,'--','LineWidth',1.8);

grid on;
box on;
set(gca,'XMinorGrid','off','YMinorGrid','off');
set(gca,'FontSize',11,'LineWidth',1);

xlabel('Tiempo, t [s]');
ylabel('Aceleracion lineal, a [mm/s^2]');
title('Validacion de aceleracion');

legend('Experimental','Modelo','Location','best');

%% ============================================================
% 28. CORRIENTE REAL VS MODELO
%% ============================================================

if MOSTRAR_CORRIENTE_TORQUE
warning('I y torque son estimaciones auxiliares con J, B y Kt supuestos; no identifican Gx.');
figure('Name','Corriente','NumberTitle','off');

plot(tExp,currentModel,'--','LineWidth',1.8);
hold on;

if any(isfinite(currentReal))
    plot(tExp,currentReal,'LineWidth',1.8);
    legend('Modelo mecanico equivalente','Experimental','Location','best');
else
    legend('Modelo mecanico equivalente','Location','best');
end

grid on;
box on;
set(gca,'XMinorGrid','off','YMinorGrid','off');
set(gca,'FontSize',11,'LineWidth',1);

xlabel('Tiempo, t [s]');
ylabel('Corriente, I [A]');
title('Corriente del motor');

%% ============================================================
% 29. TORQUE REAL ESTIMADO VS MODELO
%% ============================================================

figure('Name','Torque','NumberTitle','off');

plot(tExp,torqueModel,'--','LineWidth',1.8);
hold on;

if any(isfinite(torqueReal))
    plot(tExp,torqueReal,'LineWidth',1.8);
    legend('Modelo','Estimado desde corriente experimental','Location','best');
else
    legend('Modelo','Location','best');
end

grid on;
box on;
set(gca,'XMinorGrid','off','YMinorGrid','off');
set(gca,'FontSize',11,'LineWidth',1);

xlabel('Tiempo, t [s]');
ylabel('Torque, T [N*m]');
title('Torque estimado: revisar supuestos');
end % MOSTRAR_CORRIENTE_TORQUE

%% ============================================================
% 30. IDENTIFICACION PWM -> VELOCIDAD
%% ============================================================

figure('Name','Identificacion Kv','NumberTitle','off');
hold on;

for k = 1:numel(directions)

    d = directions(k);

    idx = AVERAGES(:,1)==d;

    pwmDir = AVERAGES(idx,2);
    vDir   = AVERAGES(idx,3);

    plot( ...
        pwmDir, ...
        vDir, ...
        'o', ...
        'MarkerSize',7, ...
        'DisplayName',sprintf('DIR %d experimental',d));

    valid = isfinite(pwmDir) & isfinite(vDir);

    pwmDirValid = pwmDir(valid);
    vDirValid   = vDir(valid);

    if numel(unique(pwmDirValid)) >= 2

        P = polyfit(pwmDirValid,vDirValid,1);

        xx = linspace(min(pwmDirValid),max(pwmDirValid),100);
        yy = polyval(P,xx);

        plot( ...
            xx, ...
            yy, ...
            'LineWidth',1.5, ...
            'DisplayName',sprintf('DIR %d ajuste',d));
    end
end

grid on;
box on;
set(gca,'XMinorGrid','off','YMinorGrid','off');
set(gca,'FontSize',11,'LineWidth',1);

xlabel('PWM fisico [%]');
ylabel('Velocidad estacionaria [mm/s]');
title('Identificacion de Kv y zona muerta');

legend('Location','best');

%% ============================================================
% 31. RESUMEN FINAL
%% ============================================================

fprintf('\n========================================\n');
fprintf('RESUMEN FINAL\n');
fprintf('========================================\n');

fprintf('Kv                  = %.8f (mm/s)/%%PWM\n',Kv);
fprintf('Tau                 = %.8f s\n',tau);
fprintf('PWM dead            = %.8f %%\n',PWM_dead);

fprintf('Kt auxiliar supuesto= %.8f N*m/A (no necesario para PID)\n',Kt);
fprintf('Ke                  = %.8f V/(rad/s)\n',Ke);

fprintf('J utilizado         = %.8e kg*m^2\n',J_EQ);
fprintf('B auxiliar supuesto = %.8e N*m*s/rad\n',B_VISCOUS);

fprintf('mm/cuenta nominal   = %.9f\n',MM_PER_COUNT_NOMINAL);
fprintf('mm/cuenta utilizado = %.9f\n',mmPerCountReal);
fprintf('NOTA: posicion del encoder no es una medicion fisica independiente.\n');


%% RESUMEN FISICO PARA LA TESIS (SIN EXPORTACION AUTOMATICA)
TablaFisica=table(FISICA.F_resorte_ref_N,FISICA.compresion_ref_mm,Cargas.k_N_m, ...
    FISICA.m_carro_kg,FISICA.m_ladrillo_kg,Cargas.T_resorte_ref_Nm, ...
    Kv,tau,PWM_dead,RELACION_ENCODER_HUSILLO, ...
    'VariableNames',{'F_resorte_referencia_N','Compresion_referencia_mm', ...
    'Rigidez_inferida_N_m','Masa_carro_kg','Masa_ladrillo_kg', ...
    'Torque_resorte_referencia_Nm','Kv_mm_s_por_pct','Tau_s','Zona_muerta_pct','Reduccion'});
disp(TablaFisica);
fprintf('Etapa declarada: %s. NaN significa dato pendiente, no cero.\n',ETAPA_ENSAYO);
fprintf('42.88 N por si solos NO determinan rigidez ni cambio de Kv/tau.\n');
fprintf('Identificar con ensayos de la misma carga; la compresion variable puede no tener Vss.\n');
fprintf('TablaFisica y Cargas.tabla quedan disponibles en Workspace.\n');
if MOSTRAR_GRAFICA_CARGAS
    figure('Name','Carga fisica adicional','NumberTitle','off');
    subplot(3,1,1); plot(tExp,Cargas.compresion_mm,'LineWidth',1.8); grid on;
    ylabel('Compresion [mm]'); title('Datos fisicos / estimaciones; no fuerza medida');
    subplot(3,1,2); plot(tExp,[Cargas.F_resorte_N,Cargas.F_axial_N],'LineWidth',1.8);
    hold on; plot(tExp,FISICA.F_resorte_ref_N*ones(size(tExp)),'k:','LineWidth',1.2);
    grid on; ylabel('Fuerza [N]');
    legend('Resorte estimado','Axial total estimada','Referencia del resorte','Location','best');
    subplot(3,1,3); plot(tExp,Cargas.T_carga_Nm,'LineWidth',1.8); grid on;
    ylabel('Torque de carga [N m]'); xlabel('Tiempo [s]');
end

%% ============================================================
% 32. GUARDAR TABLAS
%% ============================================================

if EXPORTAR_TABLAS
carpetaID = fullfile(fileparts(modelPath),"Caracterizacion_"+EJE_ID);
if ~isfolder(carpetaID), mkdir(carpetaID); end
writetable(ResultTable,fullfile(carpetaID,'ResultadosIntentos.csv'));
writetable(AverageTable,fullfile(carpetaID,'PromediosEnsayos.csv'));
writetable(Modelo.por_direccion,fullfile(carpetaID,'ModelosPorDireccion.csv'));

fprintf('\nArchivos generados:\n');
fprintf('Carpeta: %s\n',carpetaID);
fprintf('ResultadosIntentos.csv\n');
fprintf('PromediosEnsayos.csv\n');
fprintf('ModelosPorDireccion.csv\n');
end
fprintf('\nGv, Gx, Kv, tau, PWM_dead, Kt, Ke, J_EQ y B_VISCOUS quedan en Workspace.\n');

%% ============================================================
% FUNCION LOCAL 1
% LEER TXT DEL ESP32
%% ============================================================

function [DATA,META] = readSerialFile(filename)
    % Adaptador: mantiene las 10 columnas internas del programa original.
    % V2: 9 columnas, t_us, bloques START/EVENT/END.
    % Antiguo: 10 columnas, intento y t_ms, como esperaba el MATLAB original.
    fid=fopen(filename,'r');
    if fid==-1, error('No se pudo abrir "%s".',filename); end
    closeFile=onCleanup(@() fclose(fid)); %#ok<NASGU>
    DATA=zeros(0,10);
    META=struct('trial',{},'idESP',{},'tOn',{},'tOff',{});
    opened=false; sequence=0; raw=zeros(0,9); damaged=false;
    originalID=NaN; on=NaN; off=NaN; kind=''; formatSeen=0;
    while ~feof(fid)
        line=fgetl(fid);
        if ~ischar(line), break; end
        line=strtrim(line);
        if isempty(line), continue; end
        if startsWith(line,'# START,')
            if formatSeen==10, error('No mezclar formatos V2 y antiguo en un archivo.'); end
            formatSeen=9;
            if opened, warning('Bloque %d incompleto: omitido.',sequence); end
            sequence=sequence+1; opened=true; raw=zeros(0,9); damaged=false;
            originalID=str2double(fieldValue(line,'intento'));
            kind=fieldValue(line,'tipo'); on=NaN; off=NaN;
        elseif opened && startsWith(line,'# EVENT,')
            on=str2double(fieldValue(line,'on_us'))*1e-6;
            off=str2double(fieldValue(line,'off_us'))*1e-6;
        elseif opened && startsWith(line,'# END,')
            reason=fieldValue(line,'motivo');
            complete=str2double(fieldValue(line,'intento'))==originalID;
            quiet=str2double(fieldValue(line,'quiet'))==1;
            overflow=str2double(fieldValue(line,'overflow'));
            good=complete && quiet && overflow==0 && ~damaged && ...
                 strcmp(kind,'S') && strcmp(reason,'TIEMPO_COMPLETADO') && ...
                 size(raw,1)>=3 && isfinite(on) && isfinite(off) && off>on;
            if good && all(diff(raw(:,2))>0)
                times=raw(:,2)*1e-6;
                acceleration=gradient(raw(:,8),times);
                adapted=[repmat(sequence,size(raw,1),1),raw(:,2)/1000,raw(:,3:9),acceleration];
                DATA=[DATA;adapted]; %#ok<AGROW>
                META(end+1)=struct('trial',sequence,'idESP',originalID,'tOn',on,'tOff',off); %#ok<AGROW>
                fprintf('Intento MATLAB %d = intento ESP %d (S completo).\n',sequence,originalID);
            else
                warning('Bloque %d omitido: tipo=%s, motivo=%s, o datos incompletos.',sequence,kind,reason);
            end
            opened=false;
        elseif startsWith(line,'#') || startsWith(lower(line),'intento')
            continue;
        else
            parts=strsplit(line,','); values=str2double(parts); values=values(:).';
            if numel(values)==9 && all(isfinite(values))
                if ~opened, error('Datos V2 sin START: conservar metadatos # del registro.'); end
                if values(1)~=originalID, damaged=true; else, raw(end+1,:)=values; end %#ok<AGROW>
            elseif numel(values)==10 && all(isfinite(values))
                if formatSeen==9, error('No mezclar formatos V2 y antiguo.'); end
                formatSeen=10; DATA(end+1,:)=values; %#ok<AGROW>
            elseif opened && ~isempty(regexp(line,'^\d+,','once'))
                damaged=true;
            end
        end
    end
    if opened, warning('Ultimo bloque incompleto: omitido.'); end
    if formatSeen==10
        warning('Formato antiguo: sin motivo de cierre ni tiempo ON exacto; revisar cada ensayo.');
    end
end

function value=fieldValue(line,name)
    token=regexp(line,['(?:^|,)' name '=([^,\s]+)'],'tokens','once');
    if isempty(token), value=''; else, value=token{1}; end
end

%% ============================================================
% FUNCION LOCAL 2
% LIMPIAR UN INTENTO
%% ============================================================

function [Dclean, ok] = cleanTrialMatrix(D)

    ok = false;
    Dclean = [];

    if isempty(D) || size(D,2) < 10
        return;
    end

    % Conservar solo filas completamente numericas
    validRows = all(isfinite(D),2);

    D = D(validRows,:);

    if size(D,1) < 3
        return;
    end

    % Ordenar por tiempo
    [~,idxSort] = sort(D(:,2));
    D = D(idxSort,:);

    % Quitar tiempos repetidos
    [~,ia] = unique(D(:,2),'stable');
    D = D(ia,:);

    if size(D,1) < 3
        return;
    end

    % Verificar crecimiento de tiempo
    if any(diff(D(:,2))<=0)
        return;
    end

    Dclean = D;
    ok = true;
end


%% CARGAS FISICAS DEL INTENTO SELECCIONADO (MODELO INDIVIDUAL)
function C=cargasFisicas(t,x_mm,v_mm_s,a_mm_s2,F,etapa,p_mm,N)
    t=t(:); x_mm=x_mm(:); v=v_mm_s(:)/1000; a=a_mm_s2(:)/1000;
    n=numel(t); C.compresion_mm=nan(n,1); C.F_resorte_N=nan(n,1);
    C.F_inercia_N=nan(n,1); C.F_rozamiento_N=nan(n,1); C.F_axial_N=nan(n,1);
    C.k_N_m=NaN;
    if F.modo=="sujecion" && isfinite(F.compresion_ref_mm) && F.compresion_ref_mm>0
        C.k_N_m=(F.F_resorte_ref_N-F.precarga_inicial_N)/(F.compresion_ref_mm/1000);
        assert(C.k_N_m>0,'Fuerza/compresion de referencia incompatibles.');
    end
    assert(ismember(F.signo_resorte_traslado,[-1 0 1]),'Signo de resorte invalido');
    assert(F.eta_husillo>0 && F.eta_husillo<=1 && F.eta_reductor>0 && F.eta_reductor<=1, ...
        'Eficiencias fuera de rango.');
    if F.modo=="avance"
        assert(any(etapa==["sin_clasificar","avance_sin_corte","avance_con_corte","retorno"]), ...
            'Etapa de avance invalida.');
        if etapa~="sin_clasificar"
            C.F_resorte_N=zeros(n,1);
            C.F_inercia_N=F.m_carro_kg*a; % M3: m_carro YA incluye toda la carga
            C.F_rozamiento_N=F.mu_equivalente*F.m_carro_kg*9.81*tanh(v/0.001);
            if etapa=="avance_con_corte"
                fc=F.F_corte_N*tanh(v/0.001); % constante aproximada por intento
            else, fc=zeros(n,1); end
            C.F_axial_N=C.F_inercia_N+C.F_rozamiento_N+fc;
        end
    else
        assert(any(etapa==["sin_clasificar","libre","compresion","traslado","mixto"]), ...
            'Etapa de sujecion invalida.');
        esComp=false(n,1); esTras=false(n,1); esLibre=false(n,1);
        if etapa=="libre", esLibre(:)=true;
        elseif etapa=="compresion", esComp(:)=true;
        elseif etapa=="traslado", esTras(:)=true;
        elseif etapa=="mixto"
            assert(isfinite(F.t_contacto_s) && isfinite(F.t_traslado_s) && ...
                F.t_traslado_s>F.t_contacto_s,'Completar tiempos medidos de contacto y traslado.');
            esLibre=t<F.t_contacto_s;
            esComp=t>=F.t_contacto_s & t<F.t_traslado_s;
            esTras=t>=F.t_traslado_s;
        end
        C.F_resorte_N(esLibre)=0; C.compresion_mm(esLibre)=0;
        if any(esComp) && isfinite(F.t_contacto_s)
            assert(F.t_contacto_s>=t(1) && F.t_contacto_s<=t(end), ...
                'El tiempo de contacto queda fuera del intento graficado.');
            xc=interp1(t,x_mm,F.t_contacto_s,'linear');
            C.compresion_mm(esComp)=max(x_mm(esComp)-xc,0);
            if isfinite(C.k_N_m)
                C.F_resorte_N(esComp)=F.precarga_inicial_N+ ...
                    C.k_N_m*C.compresion_mm(esComp)/1000;
                C.F_resorte_N(esComp & C.compresion_mm<=0)=0;
            end
        end
        C.F_resorte_N(esTras)=F.F_resorte_ref_N; % referencia mantenida, no fuerza medida continua
        if isfinite(F.compresion_ref_mm), C.compresion_mm(esTras)=F.compresion_ref_mm; end
        masa=F.m_carro_kg*ones(n,1); masa(esTras)=F.m_carro_kg+F.m_ladrillo_kg;
        C.F_inercia_N=masa.*a;
        C.F_rozamiento_N=F.mu_equivalente*masa*9.81.*tanh(v/0.001);
        contacto=C.F_resorte_N;
        contacto(esTras)=F.signo_resorte_traslado*C.F_resorte_N(esTras);
        C.F_axial_N=C.F_inercia_N+C.F_rozamiento_N+contacto;
        C.F_axial_N(~(esComp|esTras|esLibre))=NaN;
    end
    eta=F.eta_husillo*F.eta_reductor;
    alpha=p_mm/1000/(2*pi*N); beta=p_mm/1000/(2*pi);
    C.T_carga_Nm=alpha/eta*C.F_axial_N+ ...
        (F.J_husillo_kgm2+F.J_reductor_salida_kgm2)*a/(beta*N*eta);
    C.T_resorte_ref_Nm=alpha/eta*F.F_resorte_ref_N;
    C.tabla=table(t,C.compresion_mm,C.F_resorte_N,C.F_inercia_N, ...
        C.F_rozamiento_N,C.F_axial_N,C.T_carga_Nm, ...
        'VariableNames',{'t_s','Compresion_mm','F_resorte_N','F_inercia_N', ...
        'F_rozamiento_N','F_axial_estimada_N','T_carga_estimado_Nm'});
end
