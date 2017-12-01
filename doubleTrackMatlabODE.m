function dx = doubleTrackMatlabODE(s,x,u,sSim,auxdata)

vehicle = auxdata.vehicle;

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
%States
ePsi        = x(1) ;
ey          = x(2) ;
vx          = x(3) ;
vy          = x(4) ;
r           = x(5) ;
t           = x(6) ;
omega_L1    = x(7) ;
omega_R1    = x(8) ;
omega_L2    = x(9) ;
omega_R2    = x(10);
delta       = x(11);
T           = x(12);
% X           = x(11);
% Y           = x(12);

%Control
u1 = u(:,1);
u2 = u(:,2); 
u1  = interp1(sSim,u1,s);                                                         %Assume front wheel steer only
u2   = interp1(sSim,u2,s)*5000;




%Power Train
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


%road
% k           = trackParameterInterpolation(track,s);
% k = k.curvature.meas;
k =0;


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



%Tire forces
    %Tire forces
    [fx_L1, fy_L1] = simplifiedPacejka(-Fz_L1,-alpha_L1,kappa_L1,coeffFront); %Fix because Pacejka is in ISO-W and vehicle is in SAE
    [fx_R1, fy_R1] = simplifiedPacejka(-Fz_R1,-alpha_R1,kappa_R1,coeffFront);
    [fx_L2, fy_L2] = simplifiedPacejka(-Fz_L2,-alpha_L2,kappa_L2,coeffRear);
    [fx_R2, fy_R2] = simplifiedPacejka(-Fz_R2,-alpha_R2,kappa_R2,coeffRear);

    %Plot tire
    hFig = findobj('Name','Tire Fx');
    if ~isempty(hFig)
        set(0,'CurrentFigure',hFig);
        tags = {'L1','R1','L2','R2'};
        for iPos = 1:length(tags)
            currentSA = eval(['alpha_' tags{iPos}]);
            currentSR = eval(['kappa_' tags{iPos}]);
            currentFx = eval(['fx_' tags{iPos}]);
            currentFy = eval(['fy_' tags{iPos}]);
            currentFz = eval(['Fz_' tags{iPos}]);

            hAx = findall(gcf,'Type','Axes','Tag',tags{iPos});
            hLine = findall(hAx,'Tag','cursor');
            set(hLine,'XData',currentSR,'yData',currentFx);

            hLine = findall(hAx,'Tag','tireTrace');
            slipRatio = -1:0.01:1;
            [fx,~] = simplifiedPacejka(-currentFz,currentSA,slipRatio,coeff);
            set(hLine,'XData',slipRatio,'YData',fx);

            title(hAx,sprintf('SA = %4.2fdeg, SR = %4.2f, FZ = %5.0fN',currentSA*myConstants.rad2deg,currentSR,currentFz));
        end
        drawnow
    end

    hFig = findobj('Name','Tire Fy');
    if ~isempty(hFig)
        set(0,'CurrentFigure',hFig);
        tags = {'L1','R1','L2','R2'};
        for iPos = 1:length(tags)
            currentSA = eval(['alpha_' tags{iPos}]);
            currentSR = eval(['kappa_' tags{iPos}]);
            currentFx = eval(['fx_' tags{iPos}]);
            currentFy = eval(['fy_' tags{iPos}]);
            currentFz = eval(['Fz_' tags{iPos}]);

            hAx = findall(gcf,'Type','Axes','Tag',tags{iPos});
            hLine = findall(hAx,'Tag','cursor');
            set(hLine,'XData',currentSA*myConstants.rad2deg,'yData',currentFy);

            hLine = findall(hAx,'Tag','tireTrace');
            slipAngle = (-15:0.1:15)*myConstants.deg2rad;
            [~,fy] = simplifiedPacejka(-currentFz,slipAngle,currentSR,coeff);
            set(hLine,'XData',slipAngle*myConstants.rad2deg,'YData',fy);

            title(hAx,sprintf('SA = %4.2fdeg, SR = %4.2f, FZ = %5.0fN',currentSA*myConstants.rad2deg,currentSR,currentFz));
        end
        drawnow
    end
            

%% ODE Description

%Sum forces and moment
FX = cos(delta)*(fx_L1 + fx_R1) - sin(delta)*(fy_L1 + fy_R1) + fx_L2 + fx_R2 + Fax;
FY = cos(delta)*(fy_L1 + fy_R1) + sin(delta)*(fx_L1 + fx_R1) + fy_L2 + fy_R2;


dvx_dt =  vy*r + FX/m;
dvy_dt = -vx*r + FY/m;
dr_dt = (a*(cos(delta)*(fy_R1 + fy_L1) + sin(delta)*(fx_R1 + fx_L1)) + ...
         wf*(fy_R1*sin(delta) - fx_R1*cos(delta)) - wr*fx_R2 + ...
         wf*(fx_L1*cos(delta) - fy_L1*sin(delta))  + wr*fx_L2 - b*(fy_R2 + fy_L2))/Izz;



% road dynamics dynamics
sDot = (vx.*cos(ePsi) - vy.*sin(ePsi))./(1-ey.*k);

dePsi_dt = r - k.*sDot;
dey_dt   = vx.*sin(ePsi) + vy.*cos(ePsi);


%Powertrain
domega_L1_dt = (-T_drive_L1 + reff_f*fx_L1)/Jr_f; %neg used to change to SAE coords
domega_R1_dt = (-T_drive_R1 + reff_f*fx_R1)/Jr_f;
domega_L2_dt = (-T_drive_L2 + reff_r*fx_L2)/(Jr_r);
domega_R2_dt = (-T_drive_R2 + reff_r*fx_R2)/(Jr_r);


%% All dynamics dynamics
dx = [dePsi_dt ...
      dey_dt  ...
      dvx_dt   ...
      dvy_dt   ...
      dr_dt    ...
      1        ...
      domega_L1_dt ... 
      domega_R1_dt ...
      domega_L2_dt ...
      domega_R2_dt ...
      u1 ...
      u2]'./sDot;
      
%       vx./sDot ...
%       vy./sDot]';




persistent count
if isempty(count); count = 1; end
if count == 1
    fprintf('%-5s %-5s %-5s  %-5s %-5s %-5s %-5s %-5s %-5s %-5s %-5s\n','s','vx','v','aL1','ar1','FX','FY','r','t','omega_L1','omega_R2')
end
count = count+1;
if count == 21
    count = 1;
end

fprintf('%5.1f  %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f\n',s,vx,vy,alpha_L1*myConstants.rad2deg,alpha_R1*myConstants.rad2deg,FX,FY,r*myConstants.rad2deg,t,omega_L1,omega_R2);









end
%---------------------------------------------%
% END: function bicycleContinuous.m   %
%---------------------------------------------%