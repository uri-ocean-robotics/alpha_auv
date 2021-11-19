function h = hydrodynamic_terms
m = 25; %kg
r = 5*25.4/1000; %%5 inch 
L = 55*25.4/1000; %% 55 inch

%%added mass
%%slender body theory
%%CYLINDER
rho = 1023;  %%water density
A11 = 0.1*m;
A22 = rho*pi*r^2*L;
A33 = A22;
A44 = 0;
A55 = L^3/12*rho*pi*r^2;
A66 = A55;

h.added_mass = [A11; A22; A33; A44; A55; A66];


%%hydrodynamic damping terms
v=0.75;
kv = 1.267/1000000;

Re = v*L/kv;
% Cd = 0.25; %%Drag 3-12 figure 21
Cd = 0.5; % new term: @emircem 2021-09-18
Cl = 0.94; %%Drag book 3-7 figure 8.
Xuu = -1/2*rho*Cd*pi*r^2;
Yvv = -1/2*rho*Cl*L*2*r;
Zww = Yvv; %%symmetric
%%rotation 
Kpp =0; 
Mqq = -1/32*Cl*r*L^4 *rho;
Nrr = -1/32*Cl*r*L^4 *rho;

h.damping=[-Xuu, -Yvv, -Zww, -Kpp, -Mqq, -Nrr];

clearvars -except h

end