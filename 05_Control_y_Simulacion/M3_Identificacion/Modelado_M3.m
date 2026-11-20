%% M3: identificacion EMPIRICA con carga aproximada de 10 kg, SIN corte.
% M3 opera en lazo abierto entre HOME y END; el encoder verifica que no pierda cuentas.
% Requiere Control System Toolbox.
% El modelo trabaja en magnitudes por sentido: NO corrige el encoder firmado.
% Se conserva el TXT original; las distancias estimadas NO calibran el encoder.
clear; clc; close all;
BASE_DIR = fileparts(mfilename('fullpath'));
SERIAL_FILE = fullfile(BASE_DIR,'PruebasMotor3.txt');
IDS_USAR = 1:24;              % 25-27 se excluyen de TODO el analisis
IDS_IGNORAR = [];             % ID original del TXT, por ejemplo [3 4]
IDS_VALIDACION = [];          % Ensayos buenos reservados, excluidos del ajuste
MASA_TOTAL_KG = 10;           % Aproximada; no volver a sumar el ladrillo
CPR = 1000; LEAD_MM = 8; REDUCCION = 10;
Q_MM = LEAD_MM/(CPR*REDUCCION);
TOL_ESTACIONARIO = 0.10;      % Cambio relativo entre dos tramos finales
EXPORTAR = true;
% Fila: ID, distancia referencia 4s [mm], maximo observado [A], estimada(1/0)
% Filas 1-6: distancias de pruebas DIFERENTES de 2s multiplicadas por dos.
% Sus corrientes tampoco se afirman medidas simultaneamente con este TXT.
MED = [1 20 .25 1;2 19.2 .22 1;3 22 .27 1;4 24 .28 1;5 34 .34 1;6 32 .35 1;
7 38 .44 0;8 37 .40 0;9 46 .50 0;10 47 .52 0;11 51 .60 0;12 51 .56 0;
13 60 .65 0;14 59 .69 0;15 66 .79 0;16 66 .78 0;17 74 .88 0;18 74 .86 0;
19 78 .92 0;20 78 .91 0;21 85 1.05 0;22 84 1.08 0;23 89 1.16 0;24 89 1.17 0];
E = leerM3(SERIAL_FILE);
E = E(ismember([E.id],IDS_USAR) & ~ismember([E.id],IDS_IGNORAR));
assert(~isempty(E),'No hay ensayos seleccionados.');
assert(numel(unique([E.id]))==numel(E),'IDs repetidos: separar las sesiones.');
R = nan(numel(E),13);
for j=1:numel(E)
    a=E(j).a; t=a(:,2)*1e-6; on=E(j).on; off=E(j).off;
    assert(abs(E(j).q-Q_MM)<1e-10,'Conversion mm/cuenta incompatible.');
    assert(abs(E(j).N-REDUCCION)<1e-8 && abs(E(j).cpr-CPR)<1e-8, ...
        'Reductor o cuentas del registro incompatibles.');
    dx=a(:,6)-a(1,6); signo=sign(dx(end));
    if signo==0, signo=1; end
    x=dx*Q_MM*signo; v=a(:,8)*signo;
    assert(max(abs((a(:,7)-a(1,7))-dx*Q_MM))<0.02, ...
        'Columnas x_mm y delta_counts incompatibles.');
    % Normalizacion local explicita: solo para identificacion en magnitud.
    E(j).t=t; E(j).x=x; E(j).v=v;
    dur=off-on;
    act=t>=on & t<off;
    cola=t>=on+.70*dur & t<off;
    c1=t>=on+.60*dur & t<on+.80*dur;
    c2=t>=on+.80*dur & t<off;
    assert(nnz(cola)>=6 && nnz(c1)>=3 && nnz(c2)>=3,'Escalon demasiado corto.');
    Vss=median(v(cola)); pwm=median(abs(a(act,3)));
    variacion=abs(median(v(c1))-median(v(c2)))/max(abs(Vss),eps);
    previo=t>=max(t(1),on-.10) & t<on;
    v0=median(v(previo));
    tau=NaN; target=v0+.63212056*(Vss-v0);
    k=find(act & v>=target,1,'first');
    if ~isempty(k) && k>1 && v(k)>v(k-1)
        tc=t(k-1)+(target-v(k-1))*(t(k)-t(k-1))/(v(k)-v(k-1));
        tau=tc-on; % Tiempo equivalente desde orden ON; puede incluir retardo.
    end
    ok=E(j).ok && Vss>.1 && isfinite(tau) && tau>0 && variacion<=TOL_ESTACIONARIO;
    mi=find(MED(:,1)==E(j).id,1); dist=NaN; imax=NaN; estimada=NaN;
    if ~isempty(mi), dist=MED(mi,2); imax=MED(mi,3); estimada=MED(mi,4); end
    R(j,:)=[E(j).id E(j).dir pwm dur x(end) dist imax estimada Vss tau variacion signo ok];
end
Resultados=array2table(R,'VariableNames',{'ID','DIR','PWM_pct','Duracion_s', ...
    'Encoder_mm','Referencia_mm','Imax_observado_A','Referencia_estimada', ...
    'Vss_mm_s','Tau63_s','Cambio_cola_rel','Signo_encoder','Candidato'});
disp(Resultados);
if numel(unique(R(:,12)))==1 && numel(unique(R(:,2)))>1
    warning(['Encoder con el mismo signo en ambas direcciones. Modelos de magnitud ' ...
        'provisionales; comprobar A/B antes de control real de posicion.']);
end
out=fullfile(BASE_DIR,'Resultados_M3');
if EXPORTAR
    if ~isfolder(out), mkdir(out); end
    writetable(Resultados,fullfile(out,'Resumen_ensayos.csv'));
end
Modelos=cell(2,1);
for z=1:2
    direccion=3-2*z; % +1, -1
    sel=R(:,2)==direccion & R(:,13)==1 & ~ismember(R(:,1),IDS_VALIDACION);
    assert(nnz(sel)>=3,'Faltan ensayos validos para DIR=%d.',direccion);
    % Promediar por nivel real de PWM para no ponderar mas las repeticiones.
    niveles=unique(R(sel,3)); vv=zeros(size(niveles));
    for n=1:numel(niveles), vv(n)=mean(R(sel & R(:,3)==niveles(n),9)); end
    assert(numel(niveles)>=3,'Se necesitan al menos tres niveles de PWM.');
    p=polyfit(niveles,vv,1); Kv=p(1); dead=-p(2)/p(1);
    assert(Kv>0 && dead>=0 && dead<min(niveles),'Ajuste lineal/zona muerta no admisible.');
    r2=1-sum((vv-polyval(p,niveles)).^2)/max(sum((vv-mean(vv)).^2),eps);
    tau=median(R(sel,10)); % Valor representativo; mostrar tambien variacion.
    if max(R(sel,10))/min(R(sel,10))>1.5
        warning('DIR=%d: tau cambia con PWM; revisar modelo en rango de uso.',direccion);
    end
    if r2<.95, warning('Ajuste PWM-velocidad R2 bajo: %.4f.',r2); end
    if direccion==1, nombre='M3_DIR1'; else, nombre='M3_DIRmenos1'; end
    s=tf('s');
    Modelo=struct('eje',string(nombre),'Kv',Kv,'tau',tau,'PWM_dead',dead, ...
        'mm_por_cuenta',Q_MM,'DIR_MODELO',direccion,'masa_total_kg',MASA_TOTAL_KG, ...
        'masa_aproximada',true,'condicion',"sin corte",'CPR',CPR, ...
        'LEAD_mm',LEAD_MM,'relacion_encoder_husillo',REDUCCION, ...
        'fuente',"TXT: ajuste Vss-PWM y mediana de tiempos al 63.2 por ciento", ...
        'archivo',SERIAL_FILE,'validacion_aprobada',false,'signo_encoder_validado',false, ...
        'rango_PWM',[min(niveles) max(niveles)],'R2',r2, ...
        'tau_por_ensayo',R(sel,[1 10]),'Gv',Kv/(tau*s+1),'Gx',Kv/(s*(tau*s+1)));
    Modelo.ids_identificacion=R(sel,1).';
    Modelo.ids_validacion=R(R(:,2)==direccion & ismember(R(:,1),IDS_VALIDACION),1).';
    Modelo.tiene_validacion_reservada=~isempty(Modelo.ids_validacion);
    Modelo.nota_zona_muerta='Extrapolacion lineal; no equivale al PWM minimo de arranque.';
    Modelo.nota_tau='Tiempo equivalente al 63.2% desde ON; no separa retardo puro.';
    Modelos{z}=Modelo;
    warning(['Zona muerta extrapolada de 20-75%%; no prueba arranque a bajo PWM.']);
    fprintf('\n%s: Kv=%.6f mm/s/%%, tau=%.6f s, dead=%.4f %%, R2=%.5f\n', ...
        nombre,Kv,tau,dead,r2);
    figure('Name',['Identificacion ' nombre]);
    subplot(2,1,1); plot(niveles,vv,'o',niveles,polyval(p,niveles),'-');
    grid on; xlabel('PWM [%]'); ylabel('Velocidad [mm/s]'); legend('Medida','Ajuste','Location','best');
    subplot(2,1,2); plot(R(sel,3),R(sel,10),'o'); grid on;
    xlabel('PWM [%]'); ylabel('Tiempo equivalente 63.2% [s]');
    % Comparar TODOS los ensayos candidatos seleccionados del sentido.
    indices=find(R(:,2)==direccion & R(:,13)==1);
    errores=zeros(numel(indices),4);
    elegido=indices(ceil(numel(indices)/2));
    for k=1:numel(indices)
        j=indices(k); t=E(j).t; dt=median(diff(t));
        tu=(t(1):dt:t(end)).';
        % PWM constante cuantizado delimitado por EVENT, evita adelantar ON.
        ue=max(R(j,3)-dead,0)*double(tu>=E(j).on & tu<E(j).off);
        planta=ss([0 1;0 -1/tau],[0;Kv/tau],eye(2),zeros(2,1));
        y=lsim(planta,ue,tu,[0;0]);
        ys=interp1(tu,y,t,'linear','extrap');
        errores(k,:)=[R(j,1),sqrt(mean((ys(:,1)-E(j).x).^2)), ...
            sqrt(mean((ys(:,2)-E(j).v).^2)),ismember(R(j,1),IDS_VALIDACION)];
        if j==elegido
            figure('Name',['Comparacion ' nombre]);
            subplot(2,1,1); plot(t,E(j).x,t,ys(:,1)); grid on;
            ylabel('Desplazamiento [mm]'); legend('Encoder','Modelo','Location','best');
            title(sprintf('Ensayo %d: comparacion; no implica validacion aprobada',R(j,1)));
            subplot(2,1,2); plot(t,E(j).v,t,ys(:,2)); grid on;
            ylabel('Velocidad [mm/s]'); xlabel('Tiempo [s]'); legend('Encoder','Modelo','Location','best');
        end
    end
    Errores=array2table(errores,'VariableNames',{'ID','RMSE_x_mm','RMSE_v_mm_s','Reservado'});
    disp(Errores);
    if EXPORTAR
        save(fullfile(BASE_DIR,['Modelo_' nombre '.mat']),'Modelo');
        writetable(Errores,fullfile(out,['Errores_' nombre '.csv']));
    end
end
figure('Name','Mediciones complementarias');
for z=1:2
    d=3-2*z; ix=R(:,2)==d;
    subplot(2,1,1); hold on; plot(R(ix,3),R(ix,7),'-o','DisplayName',sprintf('DIR %d',d));
end
grid on; xlabel('PWM [%]'); ylabel('Maximo observado [A]'); legend('show');
subplot(2,1,2); plot(R(:,1),R(:,5),'k.-','DisplayName','Encoder'); hold on;
ix=R(:,8)==0; plot(R(ix,1),R(ix,6),'bo','DisplayName','Medicion fisica');
ix=R(:,8)==1; plot(R(ix,1),R(ix,6),'rx','DisplayName','Estimacion 2s a 4s');
grid on; xlabel('ID'); ylabel('Distancia final [mm]'); legend('show');
fprintf('\nModelos provisionales: la masa no se suma otra vez a la planta identificada.\n');
fprintf('No se estima torque desde Imax de bus ni se simula fuerza de corte.\n');

function E=leerM3(file)
% Lector estricto del formato v3 de diez columnas. No interpreta datos viejos.
fid=fopen(file,'r'); assert(fid~=-1,'No se pudo abrir %s.',file);
c=onCleanup(@() fclose(fid)); %#ok<NASGU>
E=struct([]); abierto=false; b=struct();
while ~feof(fid)
    line=strtrim(fgetl(fid));
    if startsWith(line,'# START,')
        assert(~abierto,'Bloque sin END antes del siguiente START.');
        b=struct('id',numero(line,'intento'),'dir',numero(line,'dir'), ...
            'tipo',campo(line,'tipo'),'a',zeros(0,10),'on',NaN,'off',NaN, ...
            'q',NaN,'N',NaN,'cpr',NaN,'cabecera',false,'ok',false);
        abierto=true;
    elseif abierto && startsWith(line,'# CONFIG,')
        b.q=numero(line,'mm_count'); b.N=numero(line,'reductor'); b.cpr=numero(line,'cpr_motor');
    elseif abierto && startsWith(line,'# EVENT,')
        b.on=numero(line,'on_us')*1e-6; b.off=numero(line,'off_us')*1e-6;
    elseif abierto && startsWith(line,'intento,t_us,')
        assert(strcmp(line,['intento,t_us,pwm,dir,counts,delta_counts,x_mm,' ...
            'v_mm_s,rpm_salida,vueltas_salida']),'Cabecera inesperada.');
        b.cabecera=true;
    elseif abierto && startsWith(line,'# END,')
        assert(numero(line,'intento')==b.id,'ID END incompatible.');
        assert(b.cabecera && size(b.a,1)>=3 && all(diff(b.a(:,2))>0),'Datos incompletos o tiempos invalidos.');
        assert(isfinite(b.on) && isfinite(b.off) && b.off>b.on,'EVENT invalido.');
        assert(all(b.a(:,1)==b.id) && all(b.a(:,4)==b.dir),'ID/DIR de filas incompatible.');
        b.ok=strcmp(b.tipo,'S') && strcmp(campo(line,'motivo'),'TIEMPO_COMPLETADO') ...
            && numero(line,'overflow')==0 && numero(line,'input_overflow')==0 && numero(line,'quiet')==1;
        if isempty(E), E=b; else, E(end+1)=b; end %#ok<AGROW>
        abierto=false;
    elseif abierto && ~isempty(regexp(line,'^\d+,','once'))
        v=str2double(strsplit(line,','));
        assert(numel(v)==10 && all(isfinite(v)),'Fila numerica invalida.');
        b.a(end+1,:)=v;
    end
end
assert(~abierto,'Ultimo bloque incompleto: no usar registro truncado.');
end
function v=numero(s,key)
v=str2double(campo(s,key));
end
function v=campo(s,key)
r=regexp(s,['(?:^|,)' key '=([^,\s]+)'],'tokens','once');
if isempty(r), v=''; else, v=r{1}; end
end
