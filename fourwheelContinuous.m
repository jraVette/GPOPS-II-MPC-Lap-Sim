%---------------------------------------------%
% BEGIN: function bicycleContinuous.m %
%---------------------------------------------%
function phaseout = fourwheelContinuous(input)

%Vehicle parameters
track  = input.auxdata.track;
vehicle = input.auxdata.vehicle;

%Tire Parameters
coeffFront          = vehicle.tire_front.coeff.meas;     
coeffRear           = vehicle.tire_rear.coeff.meas;    

%Vehicle prameters
g = 9.81;
m                   = vehicle.parameter.mass.meas;                         %Total vehicle mass                             [kg] 
Izz                 = vehicle.parameter.yawInertia.meas;                   %Yaw Inertia                                    [kg*m^2]
a                   = vehicle.parameter.a.meas ;                           %Length of front axle to CG                     [m]
b                   = vehicle.parameter.b.meas;                            %Length of rear axle to CG                      [m]
h                   = vehicle.parameter.hcg.meas;                          %Height of CG                                   [m]
D                   = vehicle.parameter.rollStiffnessDistribution.meas;    %Roll stiffness distribution of the front axle                           [m]
reff_f              = vehicle.tire_front.reff.meas;                        %Effective rolling radius front axle            [m]  
reff_r              = vehicle.tire_rear.reff.meas;                         %Effective rolling radius front axle            [m]  
kd                  = vehicle.parameter.differentialFrictionCoeff.meas;    %Differential friction coeff                    [N*m*s/rad]
CD                  = vehicle.parameter.coeffDrag.meas;                    %Coefficient of drag
CL                  = vehicle.parameter.coeffLift.meas;                    %Coefficient of lift
frontalArea         = vehicle.parameter.frontalArea.meas;                  %Frontal area                                   [m^2]
rho                 = vehicle.parameter.airDensity.meas;                   %Air Density                                    [kg/m^3]
a_a                 = vehicle.parameter.a_a.meas;                          %Dist to CP to front axle                       [m]                                         %Accel due to gravity                           [m/s^2]
wf                  = vehicle.parameter.trackWidth_front.meas/2;           %1/2 of front track                             [m]
wr                  = vehicle.parameter.trackWidth_rear.meas/2;            %1/2 of rear track                              [m]
Jtire               = vehicle.tire_front.tireInertia.meas;     
Jwheel              = vehicle.tire_front.wheelInertia.meas;
Jr_f                = Jtire + Jwheel;                                      %Wheel inertia front wheel+tire                 [kg*m^2]
Jtire               = vehicle.tire_rear.tireInertia.meas;     
Jwheel              = vehicle.tire_rear.wheelInertia.meas;
Jr_r                = Jtire + Jwheel;                                      %Wheel inertia rear wheel+tire                  [kg*m^2]
torqueBrakingRear   = vehicle.parameter.torqueDistBrakingRear.meas;        %Torque distrubution going to rear under braking [-] \in [0,1]
torqueDrivingRear   = vehicle.parameter.torqueDistDrivingRear.meas;        %Torque distrubution going to rear under driving [-] \in [0,1]
maxEnginePower      = vehicle.parameter.enginePower.meas*745.7010; %Max engine power                                [W]




%% Set up states
%Extract state, control, and parameters for problems
s    = input.phase.time;

%States
ePsi        = input.phase.state(:,1);
ey          = input.phase.state(:,2);
vx          = input.phase.state(:,3);
vy          = input.phase.state(:,4);
r           = input.phase.state(:,5);
t           = input.phase.state(:,6);
omega_L1    = input.phase.state(:,7);
omega_R1    = input.phase.state(:,8);
omega_L2    = input.phase.state(:,9);
omega_R2    = input.phase.state(:,10);
delta       = input.phase.state(:,11);
T           = input.phase.state(:,12);
% fz_L1       = input.phase.state(:,11);
% fz_R1       = input.phase.state(:,12);
% fz_L2       = input.phase.state(:,13);
% fz_R2       = input.phase.state(:,14);

%Control
u1  = input.phase.control(:,1);
u2   = input.phase.control(:,2);

%% Power Train
%Power Train (based on the work in Tremlett)
tPlus   = 0.5+0.5*sin(atan(100*T));
tMinus  = 0.5-0.5*sin(atan(100*T));
kt = tPlus*torqueDrivingRear + tMinus*torqueBrakingRear;

T_drive_L1 = (1-kt).*(T)/(2);
T_drive_R1 = (1-kt).*(T)/(2);
T_drive_L2 = (kt).*(T)/(2) + kd*(omega_L2 - omega_R2);
T_drive_R2 = (kt).*(T)/(2) - kd*(omega_L2 - omega_R2);

%Aero loads      
Faz =  0.5*CL*rho*frontalArea*vx.^2;
Fax = -0.5*CD*rho*frontalArea*vx.^2;

FxSS = T_drive_L1/reff_f + T_drive_R1/reff_f + T_drive_L2/reff_r + T_drive_R2/reff_r;
FySS = vx.*r*m;

%Loads (based on teh work in Tremlett)
Fz_L1       = (1/2)*b*(-g*m-Faz)/(b+a)-(1/2)*D*h*FySS/(D*wf-D*wr+wr)-(1/2)*(-h*FxSS-(a_a-a)*Faz)/(b+a);
Fz_R1       = (1/2)*b*(-g*m-Faz)/(b+a)+(1/2)*D*h*FySS/(D*wf-D*wr+wr)-(1/2)*(-h*FxSS-(a_a-a)*Faz)/(b+a);
Fz_L2       = (1/2)*a*(-g*m-Faz)/(b+a)+(1/2)*(D-1)*h*FySS/(D*wf-D*wr+wr)+(1/2)*(-h*FxSS-(a_a-a)*Faz)/(b+a);
Fz_R2       = (1/2)*a*(-g*m-Faz)/(b+a)-(1/2)*(D-1)*h*FySS/(D*wf-D*wr+wr)+(1/2)*(-h*FxSS-(a_a-a)*Faz)/(b+a);

%Road
%k = zeros(size(s));
k = interp1(track.distance.meas,track.curvature.meas,s,'spline','extrap');

%% Kinematics

%Slip Ratios 
kappa_L1    = -(1 + reff_f*omega_L1./(cos(delta).*(vx + r*wf) + sin(delta).*(r*a + vy))); 
kappa_R1    = -(1 + reff_f*omega_R1./(cos(delta).*(vx - r*wf) + sin(delta).*(r*a + vy)));
kappa_L2    = -(1 + reff_r*omega_L2./(vx + r*wr)); 
kappa_R2    = -(1 + reff_r*omega_R2./(vx - r*wr));                                     

%Slip angles:
alpha_L1    = atan((-sin(delta).*(vx + r*wf) + cos(delta).*(vy + r*a))./(cos(delta).*(vx + r*wf) + sin(delta).*(vy + r*a)));
alpha_R1    = atan(( sin(delta).*(r*wf - vx) + cos(delta).*(vy + r*a))./(cos(delta).*(vx - r*wf) + sin(delta).*(vy + r*a)));
alpha_L2    = atan((vy - r*b)./(vx + r*wr));
alpha_R2    = atan((vy - r*b)./(vx - r*wr));


%% Tire forces

    
    %Tire forces
    [fx_L1, fy_L1] = simplifiedPacejka(-Fz_L1,-alpha_L1,kappa_L1,coeffFront); %Fix because Pacejka is in ISO-W and vehicle is in SAE
    [fx_R1, fy_R1] = simplifiedPacejka(-Fz_R1,-alpha_R1,kappa_R1,coeffFront);
    [fx_L2, fy_L2] = simplifiedPacejka(-Fz_L2,-alpha_L2,kappa_L2,coeffRear);
    [fx_R2, fy_R2] = simplifiedPacejka(-Fz_R2,-alpha_R2,kappa_R2,coeffRear);

%Sum forces and moment
FX = cos(delta).*(fx_L1 + fx_R1) - sin(delta).*(fy_L1 + fy_R1) + fx_L2 + fx_R2 + Fax;
FY = cos(delta).*(fy_L1 + fy_R1) + sin(delta).*(fx_L1 + fx_R1) + fy_L2 + fy_R2;


dvx_dt =  vy.*r + FX/m;
dvy_dt = -vx.*r + FY/m;
dr_dt = (a*(cos(delta).*(fy_R1 + fy_L1) + sin(delta).*(fx_R1 + fx_L1)) + ...
         wf*(fy_R1.*sin(delta) - fx_R1.*cos(delta)) - wr*fx_R2 + ...
         wf*(fx_L1.*cos(delta) - fy_L1.*sin(delta))  + wr*fx_L2 - b*(fy_R2 + fy_L2))/Izz;



%% road dynamics dynamics
sDot = (vx.*cos(ePsi) - vy.*sin(ePsi))./(1-ey.*k);

dePsi_dt = r - k.*sDot;
dey_dt   = vx.*sin(ePsi) + vy.*cos(ePsi);

%Powertrain
domega_L1_dt = (-T_drive_L1 + reff_f*fx_L1)/Jr_f; %neg used to change to SAE coords
domega_R1_dt = (-T_drive_R1 + reff_f*fx_R1)/Jr_f;
domega_L2_dt = (-T_drive_L2 + reff_r*fx_L2)/(Jr_r);
domega_R2_dt = (-T_drive_R2 + reff_r*fx_R2)/(Jr_r);

%% All dynamics dynamics 
          % dePsi_dt       dey_dt       dvx_dt       dvy_dt       dr_dt        dt_ds   domega_L1_ds       domega_L1_ds       domega_R1_ds       domega_L2_ds domega_R2_ds    ddelta_ds   dT_ds
dynamics = [dePsi_dt./sDot dey_dt./sDot dvx_dt./sDot dvy_dt./sDot dr_dt./sDot  1./sDot domega_L1_dt./sDot domega_R1_dt./sDot domega_L2_dt./sDot domega_R2_dt./sDot           u1./sDot    u2./sDot     ];
phaseout.dynamics  = dynamics;
%   phaseout.path      = [max(-omega_L1,zeros(size(omega_L1))).*max(-omega_R1,zeros(size(omega_L1))).*(fx_L1 - fx_R1),...
%                         reff_r*(fx_L2 - fx_R2) + kd*(omega_L2 - omega_R2)];
% phaseout.integrand = [dey_dt./sDot dvx_dt./sDot dvy_dt./sDot dr_dt./sDot u1./sDot u2./sDot ];
wheelSpeed = abs((omega_L2+omega_R2))/2; %Need positive number
phaseout.path = (T).*wheelSpeed - (maxEnginePower);
                 %Torque [N*m]*[rad/s]       [W]    
                 %Power [N*m/s = W]





end
%---------------------------------------------%
% END: function bicycleContinuous.m   %
%---------------------------------------------%
