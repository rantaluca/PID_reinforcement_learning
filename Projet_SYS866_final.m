%% Constantes et conditions initiales
Ts = 0.1 ; %temps d'échantillonage
V=20;      % m^3 (volume chambre)
p_atm = 101325;    % Pa (pression atmosphérique)
rho_air = 1.2;    % kg/m^3 (masse volumique de l'air)

%% Fuites 

rng('shuffle');

% Tirage de n (normal)
mu_n = 0.6;
sigma_n = 0.1;
n = mu_n + sigma_n*randn();

% Tirage de K (log-normal)
muK = -10.4;
sigmaK = 0.6;
Z = randn();    
K = exp(muK + sigmaK*Z);  

% Aire minimale (fenêtre intacte)
dP_ref=1000;      
coef=sqrt(rho_air) * K / sqrt(2);
A_min = coef * abs(dP_ref)^(n-0.5);


% Aire maximale (fenêtre très déformée)
mu_factor = 5;
sigma_factor = 2;
factor = max(0 , mu_factor + sigma_factor*randn());
A_max = min(0.01 , A_min * factor);
    
rapport = A_min / A_max;

% vitesse 
mu_v = 0.5;
sigma_v = 0.5;
v_grow = max(0 , mu_v + sigma_v*randn());

%% Ventilateur / moteur
tau_fan = 1;
saturation = 0.2; % débit massique max du ventilo en [kg/s]


%% Consigne de pression
Pc = 1000 ; %variable ou pas 
% Pression seuil 
P_seuil = 0.5*Pc;  
Pc_ref = 1 / Pc;

%% PID
%appliquer les politiques via python
Kp =0.40; 
Ki =0.040; 
Kd =0.0; 

