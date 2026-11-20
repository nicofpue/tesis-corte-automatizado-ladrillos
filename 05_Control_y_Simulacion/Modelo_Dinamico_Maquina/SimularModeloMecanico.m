function R=SimularModeloMecanico(tipo,P,carpeta)
% Modelo reducido no lineal, SI, integracion RK4 y control discreto.
% M1/M2 se resuelven juntos porque comparten UN ladrillo.
% Solo sujecion: M3 opera en lazo abierto y se identifica en M3_Identificacion.
if nargin<3, carpeta=fullfile(fileparts(mfilename('fullpath')),'Resultados_'+P.modo); end
validar(P,tipo); if ~isfolder(carpeta), mkdir(carpeta); end
assert(tipo=="sujecion",'Tipo invalido: solo "sujecion"'); R=laterales(P);
R.parametros=P; R.validado=false;
exportar(R,P,carpeta,tipo);
end

function R=laterales(P)
A=P.motor(1:2); S=P.sujecion; dt=P.Ts;
% q1 y q2: posiciones globales proyectadas, restando offsets de ancho del ladrillo.
% delta1=max(q1-y,0), delta2=max(y-q2,0). y: traslacion del ladrillo rigido.
q0=[-S.holgura_inicial,S.holgura_inicial];
z=[q0(1);0;q0(2);0;0;0;0;0]; % q1,v1,q2,v2,y,vy,Fa1,Fa2
ref=q0; C=repmat(struct('I',0,'D',0,'previo',0),1,2);
for i=1:2, C(i).previo=q0(i); end
fase=1; confirmado=0; tCambio=NaN; refCambio=ref;
tmax=S.tmax_compresion+S.t_movimiento+S.t_mantenimiento;
n=ceil(tmax/dt)+1; L=zeros(n,23); motivo="Tiempo maximo";
for k=1:n
    t=(k-1)*dt; [dz,fs,delta]=rhsL(z,[0 0],P);
    sw=delta>=[A.delta_switch];
    if fase==1
        if all(sw), confirmado=confirmado+dt; else, confirmado=0; end
        if confirmado>=S.confirmacion_s
            fase=2; tCambio=t; refCambio=ref;
        elseif t>=S.tmax_compresion
            motivo="Timeout: no se confirmaron ambos switches";
            L(k,:)=filaL(t,fase,z,ref,[0 0],fs,delta,sw,dz); break;
        end
    end
    if fase==1
        % Avanzar solo el carro cuyo switch aun no confirma compresion.
        if k>1
            ref=ref+S.vel_aproximacion*dt*[double(~sw(1)),-double(~sw(2))];
        end
    else
        d=perfil(t-tCambio,S.t_movimiento,S.desplazamiento);
        ref=refCambio+[d d]; % MISMA traslacion global en ambos carros
    end
    pwm=zeros(1,2); u=zeros(1,2);
    for i=1:2
        qi=z(2*i-1); paso=A(i).p/(A(i).CPR*A(i).N);
        medida=q0(i)+round((qi-q0(i))/paso)*paso;
        [pwm(i),u(i),C(i)]=pidPaso(ref(i),medida,C(i),A(i),dt);
    end
    L(k,:)=filaL(t,fase,z,ref,pwm,fs,delta,sw,dz);
    if any(delta>[A.delta_max]), motivo="Limite de compresion"; break; end
    if any([z(1),z(3)]<[A.q_min]) || any([z(1),z(3)]>[A.q_max])
        motivo="Limite de recorrido"; break;
    end
    if fase==2 && any(delta<=0)
        motivo="Perdida de contacto durante traslado"; break;
    end
    if fase==2 && t>=tCambio+S.t_movimiento+S.t_mantenimiento
        motivo="Compresion y traslado completados"; break;
    end
    z=integrar(@(a)rhsL(a,u,P),z,dt,P.subpasos);
    assert(all(isfinite(z)) && max(abs(z))<1e8,'Integracion divergente');
end
L=L(1:k,:);
R.tipo="sujecion_acoplada"; R.motivo=motivo; R.t_cambio=tCambio;
R.crudo=L; R.M1=tablaMotor(L,1,A(1),q0(1),P); R.M2=tablaMotor(L,2,A(2),q0(2),P);
R.ladrillo=table(L(:,1),L(:,2),1000*L(:,7),1000*L(:,8),L(:,23), ...
    L(:,15),L(:,16),L(:,15)-L(:,16), ...
    S.mu*S.m_ladrillo*P.g*tanh(L(:,8)/P.v_friccion)+S.b_ladrillo*L(:,8), ...
    'VariableNames',{'t_s','Etapa','y_mm','v_mm_s','a_m_s2', ...
    'F_M1_N','F_M2_N','F_neta_contacto_N','F_rozamiento_N'});
end

function fila=filaL(t,fase,z,ref,pwm,fs,delta,sw,dz)
fila=[t,fase,z.',ref,pwm,fs,delta,double(sw),dz(2),dz(4),dz(6)];
end

function [dz,fs,delta]=rhsL(z,u,P)
A=P.motor(1:2); S=P.sujecion;
delta=[max(z(1)-z(5),0),max(z(5)-z(3),0)];
vrel=[z(2)-z(6),z(6)-z(4)]; fs=zeros(1,2);
for i=1:2
    if delta(i)>0
        fs(i)=max(0,A(i).F_precarga+A(i).k*delta(i)+A(i).c_resorte*vrel(i));
    end
end
f1=friccion(z(2),A(1),P); f2=friccion(z(4),A(2),P);
fB=S.mu*S.m_ladrillo*P.g*tanh(z(6)/P.v_friccion)+S.b_ladrillo*z(6);
a1=(z(7)-f1-fs(1))/masa(A(1));
a2=(z(8)-f2+fs(2))/masa(A(2));
aB=(fs(1)-fs(2)-fB)/S.m_ladrillo;
dz=[z(2);a1;z(4);a2;z(6);aB; ...
    (lim(A(1).Ku*u(1),A(1).F_max)-z(7))/A(1).tau_driver; ...
    (lim(A(2).Ku*u(2),A(2).F_max)-z(8))/A(2).tau_driver];
end

function T=tablaMotor(L,i,A,q0,P)
qi=3+2*(i-1); vi=qi+1; Fi=8+i; ri=10+i; ui=12+i;
si=14+i; di=16+i; swi=18+i; ai=20+i;
q=L(:,qi); v=L(:,vi); a=L(:,ai); fuerza=L(:,Fi);
if i==1, carga=L(:,si); else, carga=-L(:,si); end
fRoz=friccion(v,A,P); fH=A.m_carro*a+fRoz+carga;
alpha=A.p/(2*pi*A.N); beta=A.p/(2*pi);
% Estimacion en accionamiento directo (motoring), no ley de retroaccion/frenado.
Tm=alpha/A.eta*fH+A.J_motor*a/alpha+ ...
    (A.J_husillo+A.J_reductor_salida)*a/(beta*A.N*A.eta);
T=table(L(:,1),L(:,2),1000*(q-q0),1000*(L(:,ri)-q0),1000*v,a, ...
    round((q-q0)*A.CPR*A.N/A.p),L(:,ui),fuerza,L(:,si),1000*L(:,di), ...
    L(:,swi),fH,masa(A)*a,fRoz,Tm, ...
    'VariableNames',{'t_s','Etapa','x_carro_mm','Referencia_mm','v_mm_s','a_m_s2', ...
    'Cuentas_encoder','PWM_pct','F_actuador_equivalente_N','F_contacto_N', ...
    'Compresion_mm','Switch','F_husillo_N','F_inercial_equivalente_N', ...
    'F_rozamiento_N','Torque_motor_estimado_Nm'});
end

function [pwm,u,C]=pidPaso(r,y,C,A,dt)
% PID sobre posicion del encoder, derivada en medicion, anti-windup.
e=r-y; C.D=A.Tf/(A.Tf+dt)*C.D-A.Kd/(A.Tf+dt)*(y-C.previo);
Umax=min(A.PWM_max-A.PWM_dead,A.F_max/A.Ku);
It=C.I+A.Ki*dt*e; ut=A.Kp*e+It+C.D;
if ~((ut>Umax && e>0)||(ut<-Umax && e<0)), C.I=lim(It,Umax); end
u=lim(A.Kp*e+C.I+C.D,Umax);
if abs(u)>1e-12, pwm=sign(u)*(abs(u)+A.PWM_dead); else, pwm=0; end
% Inversion ideal de zona muerta; caracterizar por separado en cada sentido.
u=sign(pwm)*max(abs(pwm)-A.PWM_dead,0); C.previo=y;
end

function z=integrar(f,z,dt,n)
h=dt/n;
for j=1:n
    a=f(z); b=f(z+h*a/2); c=f(z+h*b/2); d=f(z+h*c);
    z=z+h*(a+2*b+2*c+d)/6;
end
end
function m=masa(A)
alpha=A.p/(2*pi*A.N); beta=A.p/(2*pi);
m=A.m_carro+A.J_motor/alpha^2+(A.J_husillo+A.J_reductor_salida)/beta^2;
end
function f=friccion(v,A,P)
f=A.b*v+A.F_coulomb*tanh(v/P.v_friccion);
end
function y=lim(x,L), y=min(max(x,-L),L); end
function d=perfil(t,T,D)
a=min(max(t/T,0),1); d=D*(10*a^3-15*a^4+6*a^5);
end

function validar(P,tipo)
assert(P.Ts>0 && P.subpasos>=1 && mod(P.subpasos,1)==0,'Muestreo invalido');
indices=[1 2];
for i=indices
    A=P.motor(i);
    campos={'m_carro','J_motor','J_husillo','J_reductor_salida','b','F_coulomb', ...
        'Ku','tau_driver','F_max','PWM_dead','PWM_max','k','c_resorte','F_precarga'};
    for j=1:numel(campos)
        val=A.(campos{j});
        assert(isscalar(val) && isfinite(val) && val>=0, ...
            'Completar parametro %s de %s; no se sustituye por prueba.',campos{j},A.nombre);
    end
    assert(A.m_carro>0 && A.b>0 && A.Ku>0 && A.tau_driver>0 && A.F_max>0,'Parametros no positivos');
    assert(A.N>0 && A.p>0 && A.CPR>0 && A.PWM_max>A.PWM_dead && A.PWM_max<=100, ...
        'Conversion o limite PWM invalido');
    assert(A.eta>0 && A.eta<=1 && A.q_max>A.q_min,'Eficiencia/recorrido invalido');
    assert(isfinite(A.delta_switch) && A.delta_switch>0 && ...
        isfinite(A.delta_max) && A.delta_max>A.delta_switch && A.k>0, ...
        'Completar rigidez y compresion del switch/limite.');
    assert(P.Ts/P.subpasos<=A.tau_driver/5,'Reducir paso de integracion del driver');
end
assert(isfinite(P.sujecion.mu) && P.sujecion.mu>=0 && ...
    isfinite(P.sujecion.b_ladrillo) && P.sujecion.b_ladrillo>=0, ...
    'Completar rozamiento lateral.');
assert(P.sujecion.m_ladrillo>0 && P.v_friccion>0,'Masa/regularizacion invalida');
end

function exportar(R,P,carpeta,tipo)
% Los nombres y etiquetas distinguen resultados de prueba de datos medidos.
ids=[1 2];
for i=ids
    nombre="M"+i; T=R.(nombre); writetable(T,fullfile(carpeta,nombre+"_curvas.csv"));
    A=P.motor(i); alpha=A.p/(2*pi*A.N); meq=masa(A);
    % Indicadores nominales de modelo lineal LIBRE; no son una identificacion.
    Kv_teorico=1000*A.Ku/A.b; tau_mecanica=meq/A.b;
    Resumen=table(nombre,P.modo,A.N,A.p*1000,A.CPR,A.m_carro,meq, ...
        A.k,A.delta_switch*1000,A.F_precarga+A.k*A.delta_switch, ...
        Kv_teorico,tau_mecanica,max(abs(T.PWM_pct)),max(abs(T.F_husillo_N)), ...
        max(abs(T.Torque_motor_estimado_Nm)),false,string(R.motivo), ...
        'VariableNames',{'Motor','Modo','Reduccion','Avance_mm_rev','CPR', ...
        'Masa_trasladada_kg','Masa_equivalente_kg','Rigidez_N_m','Switch_mm', ...
        'Fuerza_elastica_switch_N','Kv_libre_teorico_mm_s_por_pct', ...
        'Tau_mecanica_teorica_s','MaxPWM_pct','MaxF_husillo_N', ...
        'MaxTorque_estimado_Nm','Validado','Fin_simulacion'});
    Resumen.Ku_N_por_pct=A.Ku; Resumen.Tau_driver_s=A.tau_driver;
    Resumen.Zona_muerta_pct=A.PWM_dead; Resumen.b_N_s_m=A.b;
    Resumen.F_Coulomb_N=A.F_coulomb;
    Resumen.RMSE_seguimiento_mm=sqrt(mean((T.Referencia_mm-T.x_carro_mm).^2));
    Resumen.Error_final_carro_mm=T.Referencia_mm(end)-T.x_carro_mm(end);
    Resumen.MaxVel_mm_s=max(abs(T.v_mm_s)); Resumen.MaxAceleracion_m_s2=max(abs(T.a_m_s2));
    Resumen.MaxCompresion_mm=max(T.Compresion_mm);
    writetable(Resumen,fullfile(carpeta,nombre+"_resumen.csv"));
    Modelo=struct('motor',nombre,'parametros',A,'masa_equivalente',meq, ...
        'alpha',alpha,'modo',P.modo,'validado',false, ...
        'Kv_libre_teorico',Kv_teorico,'tau_mecanica_teorica',tau_mecanica);
    Modelo.num_libre_mm_por_pct=1000*A.Ku;
    Modelo.den_libre=[A.tau_driver*meq,meq+A.tau_driver*A.b,A.b,0];
    Modelo.alcance="Planta libre teorica; no aplicar como modelo desacoplado al contacto";
    save(fullfile(carpeta,nombre+"_modelo.mat"),'Modelo','T','Resumen');
    fig=figure('Name',char(nombre+" - "+P.modo),'Color','w');
    subplot(3,1,1); plot(T.t_s,[T.x_carro_mm,T.Referencia_mm]); grid on;
    ylabel('Posicion [mm]'); legend('Carro','Referencia','Location','best');
    title(char(nombre+" - "+P.modo+" - NO validado"));
    subplot(3,1,2); plot(T.t_s,T.F_husillo_N); hold on;
    plot(T.t_s,T.F_contacto_N); legend('Husillo','Contacto');
    grid on; ylabel('Fuerza [N]');
    subplot(3,1,3); plot(T.t_s,T.PWM_pct); grid on; ylabel('PWM [%]'); xlabel('Tiempo [s]');
    exportgraphics(fig,fullfile(carpeta,nombre+"_simulacion.png"),'Resolution',160);
end
if tipo=="sujecion"
    writetable(R.ladrillo,fullfile(carpeta,'Ladrillo_curvas.csv'));
    post=R.ladrillo.Etapa==2;
    errorLateral=1000*P.sujecion.desplazamiento-R.ladrillo.y_mm(end);
    minContacto=NaN;
    if any(post), minContacto=min([R.M1.F_contacto_N(post);R.M2.F_contacto_N(post)]); end
    ResumenLadrillo=table(P.modo,R.t_cambio,1000*P.sujecion.desplazamiento, ...
        R.ladrillo.y_mm(end),errorLateral,minContacto,false,string(R.motivo), ...
        'VariableNames',{'Modo','Tiempo_compresion_s','Objetivo_traslado_mm', ...
        'Posicion_final_mm','Error_final_ladrillo_mm','MinF_contacto_traslado_N', ...
        'Validado','Fin_simulacion'});
    writetable(ResumenLadrillo,fullfile(carpeta,'Ladrillo_resumen.csv'));

    fig=figure('Name','Ladrillo y compresiones','Color','w');
    subplot(2,1,1); plot(R.ladrillo.t_s,R.ladrillo.y_mm); grid on; ylabel('Ladrillo [mm]');
    subplot(2,1,2); plot(R.M1.t_s,[R.M1.Compresion_mm,R.M2.Compresion_mm]);
    grid on; ylabel('Compresion [mm]'); xlabel('Tiempo [s]'); legend('M1','M2');
    exportgraphics(fig,fullfile(carpeta,'Ladrillo_y_resortes.png'),'Resolution',160);
end
save(fullfile(carpeta,"Simulacion_"+tipo+".mat"),'R','P');
fprintf('%s: %s. Modo=%s. Resultados NO validados.\n',tipo,R.motivo,P.modo);
end
