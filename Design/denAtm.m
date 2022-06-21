function [T,rho] = denAtm(h)
% Compute density and temperature up to 25 Km based on standard atmosphere
% model.

g0 = 9.8; % m/s^2
R = 287; % m^2/s^2K

% At surface
Ts = 288.16; % K
rhos = 1.225; %Kg/m^3
a1 = -6.5e-3; % K/m

% At base of isothermal region h=11km
rho1 = 0.3642; %Kg/m^3
h1 = 11000; % m

if h > 11000

    T = 216.66; % K constant for isothermal region
    rho = rho1 * exp(-(g0/(R*T))*(h-h1));
    
else
    
    % Gradient region 1 (0-11KM)
    T = Ts + a1*h;
    rho = rhos * (T/Ts)^(-((g0/(a1*R))+1));
    
end

end

