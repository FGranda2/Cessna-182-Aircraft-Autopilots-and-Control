function [sys,x0,str,ts] = s_funNon(t,x,u,flag)
% S-function sf_aerodyn.M
% This S-function represents the nonlinear aircraft dynamics

% Copyright 1986-2007 The MathWorks, Inc. 
%load NonLinearMats.mat

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
    otherwise
        ctrlMsgUtils.error('Controllib:general:UnexpectedError',['Unhandled flag = ',num2str(flag)]);

end

% end sfuntmpl


%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 10;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 10;
sizes.NumInputs      = 4;
%sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% Initial conditions
% x = [u;alpha;q;theta;h;v;p;r;phi;psi];
% To linearized the model assume  phi = 0 and theta = 0; 

%x0  = [0 0 0 0 0 0 0 0];
x0  = zeros(1,10);

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)
% x = [u;alpha;q;theta;h;v;p;r;phi;psi];

%NOT WITH SMALL ANGLE APPROXIMATION
a_col1 = [-0.0453126;0.0147925*x(3)-0.00433177;0.0111585-0.038105*x(3);0;0;-x(8);0;0;0;0];
a_col2 = [-67*x(3)-3.82656;-2.06302;-14.0892;0;-67;67*x(7);0;0;0;0];
a_col3 = [-67*x(2);0.0147925*x(1)-0.0204228;-0.038105*x(1)-4.34107;cos(x(9));0;0;-0.655129*x(8);-0.202341*x(7);sin(x(9))*tan(x(4));sin(x(9))/cos(x(4))];
a_col4 = [-9.81*cos(x(4));-0.145114*cos(x(9))*sin(x(4));0.37381*cos(x(9))*sin(x(4));0;67;-9.81*sin(x(9))*sin(x(4));0;0;x(8)*cos(x(9))*(tan(x(4))^2 + 1)+x(3)*sin(x(9))*(tan(x(4))^2 + 1);(x(8)*cos(x(9))*sin(x(4))/(cos(x(4))^2)) + (x(3)*sin(x(9))*sin(x(4))/(cos(x(4))^2))];
a_col5 = zeros(10,1);
a_col6 = [x(8);-0.0147925*x(7);0.038105*x(7);0;0;-0.185499;-0.454037;0.13916;0;0];
a_col7 = [0;-0.0147925*x(6);0.757108*x(8)+0.038105*x(6);0;0;67*x(2)-0.194703;-13.0948;-0.202341*x(3)-0.362479;1;0];
a_col8 = [x(6);0;0.757108*x(7);-sin(x(9));0;0.555552-x(1);2.15901-0.655129*x(3);-1.22174;cos(x(9))*tan(x(4));cos(x(9))/cos(x(4))];
a_col9 = [0;-0.145114*cos(x(4))*sin(x(9));0.37381*cos(x(4))*sin(x(9));-x(8)*cos(x(9))-x(3)*sin(x(9));0;9.81*cos(x(9))*cos(x(4));0;0;x(3)*cos(x(9))*tan(x(4))-x(8)*sin(x(9))*tan(x(4));x(3)*cos(x(9))/cos(x(4)) - x(8)*sin(x(9))/cos(x(4))];
a_col10 = zeros(10,1);

%{
%WITH SMALL ANGLE APPROXIMATION
a_col1 = [-0.0453126;0.0147925*x(3)-0.00433177;0.0111585-0.038105*x(3);0;0;-x(8);0;0;0;0];
a_col2 = [-67*x(3)-3.82656;-2.06302;-14.0892;0;-67;67*x(7);0;0;0;0];
a_col3 = [-67*x(2);0.0147925*x(1)-0.0204228;-0.038105*x(1)-4.34107;1;0;0;-0.655129*x(8);-0.202341*x(7);0;0];
a_col4 = [-9.81*cos(x(4));-0.145114*cos(x(9))*sin(x(4));0.37381*cos(x(9))*sin(x(4));0;67;-9.81*sin(x(9))*sin(x(4));0;0;0;0];
a_col5 = zeros(10,1);
a_col6 = [x(8);-0.0147925*x(7);0.038105*x(7);0;0;-0.185499;-0.454037;0.13916;0;0];
a_col7 = [0;-0.0147925*x(6);0.757108*x(8)+0.038105*x(6);0;0;67*x(2)-0.194703;-13.0948;-0.202341*x(3)-0.362479;1;0];
a_col8 = [x(6);0;0.757108*x(7);0;0;0.555552-x(1);2.15901-0.655129*x(3);-1.22174;0;1];
a_col9 = [0;-0.145114*cos(x(4))*sin(x(9));0.37381*cos(x(4))*sin(x(9));0;0;9.81*cos(x(9))*cos(x(4));0;0;0;0];
a_col10 = zeros(10,1);
%}
a = [a_col1,a_col2,a_col3,a_col4,a_col5,a_col6,a_col7,a_col8,a_col9,a_col10];
b = [0,1.7658,0,0;...
     -0.201156,0,0,0;...
     -34.9969,0,0,0;...
     0,0,0,0;...
     0,0,0,0;...
     0,0,0,5.91377;...
     0,0,-75.4745,4.84487;...
     0,0,-3.43087,-10.245;...
     0,0,0,0;...
     0,0,0,0];
sys = a*x + b*u;

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate

