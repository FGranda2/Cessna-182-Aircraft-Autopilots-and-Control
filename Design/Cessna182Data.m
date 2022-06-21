function [Long_mat,LongC_mat,Lat_mat,LatC_mat] = Cessna182Data()
% Function based on the provided file "Cessna182Config.m" to compute
% derivatives of the aircraft

%  (1) aircraft configuration parameters ****
W = 1.1788e+04;     % weight (N)
g = 9.81; 			% gravity (m/s^2)
m = W/g;	    	% mass (kg)
Ixx = 1.2683e+03;   % moment of inertia (kg m^2)
Iyy = 1.8008e+03;   % moment of inertia (kg m^2)
Izz = 2.6317e+03;   % moment of inertia (kg m^2)
Ixz = 0.0;
Izx = Ixz;
J = [ Ixx,0,-Ixz;0,Iyy,0;-Izx,0,Izz]; % inertia matrix
S = 16; 			% wing plan area (m^2)
c_bar = 1.5; 	    % mean chord (m)
b = 11; 			% wing span (m)
AR = b / c_bar;
epsilon = 0.81;     % oswald factor
k = 1 / (pi*epsilon*AR);
C_D_0 = 0.027;
C_m_0 = 0.04;
ht = 2.84; % height (m)

%  (2) flight conditions reference point ****
Ue = 67; 		% cruise velocity (m/s)
alt = 1500;     % altitude (m)
[T,rho] = denAtm(alt); % function created for standard atmospheric model
speed_of_sound = sqrt(1.4*287*T);
Mach = Ue / speed_of_sound;

%  (3) steady states
theta_e = 0;                        % pitch angle (rad)
Cw = W / (0.5 * rho * S * Ue^2);    % non-dimensional term of weight \hat{mg}
CLe = Cw * cos(theta_e); 	        % initial C_L lift coefficients 
Cze = -0.9530;                    % given by original data, not used here
CDe = 0.032; 	                    % initial C_D lift coefficients (given)
Cze = - CLe;
Cxe = Cw * sin(theta_e);
CTe = CDe + Cxe;
Cme = 0.0;

Cxu  = -0.096;          % CTu = -0.096; CDu = 0; Cxu = CTu - CDu;
Cxa  = -0.121; 		% C_{x_\alpha}
Cxq  =  0.0;
Cxdu =  0.0;			% C_{x_{\dot u}}
Cxda =  0.0; 			% C_{x_{\dot \alpha}}
Cxdq =  0.0;
Czu  = 0.0;
Cza  = -4.41;
Czq  = -3.9;
Czdu =  0.0;
Czda =  -1.7;
Czdq =  0.0;
Cmu  =  0.0;
Cma  = -0.613;
Cmq  = -12.4;
Cmdu =  0.0;
Cmda = -7.27;
Cmdq =  0.0;
Cxdelta_e = 0; %C_{x_{\delta_e}}
CLdelta_e = 0.43;   
Czdelta_e = -CLdelta_e;
Cmdelta_e = -1.122;
Cxdelta_p = 0.18 * Cw;  % this and the following delta_p are arbitrary
Czdelta_p = 0;
Cmdelta_p = 0;
Cyb = -0.393;          % C_{y_\beta}
Cyp = -0.075;
Cyr = 0.214;
Cydb = 0;               % C_{y_{\dot \beta}}
Cydp = 0;
Cydr = 0;

Clb = -0.0923;
Clp = -0.484;
Clr = 0.0798;
Cldb = 0;               % C_{l_{\dot \beta}}
Cldp = 0;
Cldr = 0;

Cnb = 0.0587;
Cnp = -0.0278;
Cnr = -0.0937;
Cndb = 0;               % C_{n_{\dot \beta}}
Cndp = 0;
Cndr = 0;

Cydelta_a = 0;          %C_{y_{\delta_a}}
Cydelta_r = 0.187;
Cldelta_a = -0.229;
Cldelta_r = 0.0147;
Cndelta_a = -0.0216;
Cndelta_r = -0.0645;

% ========= (5) dimensional derivatives ================
q = 0.5*rho*Ue^2;
qs = rho * Ue * S / 2;
qs2 = rho * Ue * Ue * S / 2;

% (5.1) Longitudinal terms
Xu  = (Cxu+2*Cxe) * qs;
Xw  = Cxa * qs;
Xq  = Cxq * qs * c_bar / 2;
Xdw = Cxda * rho * S * c_bar / 4;  
Xdu = Cxdu * rho * S * c_bar / 4;
Xdq = Cxdq * rho * S * c_bar^2 / 8;

Zu  = (Czu + 2*Cze) * qs;
Zw  = Cza * qs;
Zq  = Czq * qs * c_bar / 2;
Zdu =  Czdu * rho * S * c_bar / 4;
Zdw = Czda * rho * S * c_bar / 4;
Zdq =  Czdq * rho * S * c_bar^2 / 8;

Mu  = (Cmu + 2*Cme) * qs * c_bar;
Mw  = Cma * qs * c_bar;
Mq  = Cmq * rho * Ue * S * c_bar * c_bar / 4;
Mdu = Cmdu * rho * S * c_bar^2 / 4;
Mdw = Cmda * rho * S * c_bar^2 / 4;
Mdq = Cmdq * rho * S * c_bar^3 / 8;

Xdelta_e = Cxdelta_e * qs * Ue;
Zdelta_e = Czdelta_e * qs * Ue;
Mdelta_e = Cmdelta_e * qs * Ue * c_bar;

Xdelta_p = Cxdelta_p * qs * Ue;
Zdelta_p = Czdelta_p * qs * Ue;
Mdelta_p = Cmdelta_p * qs * Ue * c_bar;

% (5.2) Lateral terms
Yv = Cyb * qs;
Yp = Cyp * qs * b / 2;
Yr = Cyr * qs * b / 2;
Ydv = Cydb * rho * S * b / 4;
Ydp = Cydp * rho * S * b^2 / 8;
Ydr = Cydr * rho * S * b^2 / 8;

Lv = Clb * qs * b;
Lp = Clp * qs * b^2 / 2;
Lr = Clr * qs * b^2 / 2;
Ldv = Cldb * rho * S * b^2 / 4;
Ldp = Cldp * rho * S * b^3 / 8;
Ldr = Cldr * rho * S * b^3 / 8;

Nv = Cnb * qs * b;
Np = Cnp * qs * b^2 / 2;
Nr = Cnr * qs * b^2 / 2;
Ndv = Cndb * rho * S * b^2 / 4;
Ndp = Cndp * rho * S * b^3 / 8;
Ndr = Cndr * rho * S * b^3 / 8;

Ydelta_a = Cydelta_a * qs2;
Ydelta_r = Cydelta_r * qs2;
Ldelta_a = Cldelta_a * qs2 * b;
Ldelta_r = Cldelta_r * qs2 * b;
Ndelta_a = Cndelta_a * qs2 * b;
Ndelta_r = Cndelta_r * qs2 * b;

% Organize in matrices

Long_mat = [Xu,Xw,Xq,Xdu,Xdw,Xdq;...
            Zu,Zw,Zq,Zdu,Zdw,Zdq;...
            Mu,Mw,Mq,Mdu,Mdw,Mdq];
Lat_mat = [Yv,Yp,Yr,Ydv,Ydp,Ydr;...
           Lv,Lp,Lr,Ldv,Ldp,Ldr;...
           Nv,Np,Nr,Ndv,Ndp,Ndr];

% Control matrices
LongC_mat = [Xdelta_e,Xdelta_p;...
             Zdelta_e,Zdelta_p;...
             Mdelta_e,Mdelta_p];
LatC_mat = [Ydelta_a,Ydelta_r;...
            Ldelta_a,Ldelta_r;...
            Ndelta_a,Ndelta_r];
end

