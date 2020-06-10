%% Suggestion
% It would be good if the command window is maximized before running the
% simulation since there are lot of results printed.

%% Clean up
clearvars;  % clear the workspace
close all  % close all open figures
clc % clear the command window

%% Open Simulink model
open_system('rocketSim');  % open simulink model with name 'rocketSim'

%% Physical Constants
planet.g = 10; % This is the value of gravitational acceleration (m/s^2)

%% Rocket Parameters
rocket.m   = 1000;      % mass of rocket (kg)
rocket.ics = [0;0;0;0]; % initial conditions x,y,x velocity & y velocity
rocket.t1  = 5;         % Burn time for rocket in seconds after which there will not be any thrust.

%% Thrust Input Parameters
thr.F = 15000; % thrust amplitude (N)
thr.tht = 56;  % thrust angle (deg)
thr.w   = 2*pi; % thrust angle frequency (rad/s) (For scenario 2 only)
thr.amp = 20; % amplitude of time varying component of thrust ang (deg) (For scenario 2 only)

%% Load Bus Defs
load busDefs.mat  % extract & load the bus definitions in workspace

%% Solver Parameters
simPrm.h      = 0.0001; % This is time step value in seconds.
simPrm.solTyp = 'Fixed-step'; % This is the integration solver type either fixed step or variable step
simPrm.sol    = 'ode3'; % define integration method as an input for simulink solver

%% Set Simulink Configuration
set_param('rocketSim','SolverType',simPrm.solTyp);  % set this solver type in simulink parameters
set_param('rocketSim','Solver'    ,simPrm.sol);     % set this integration method in simulink solver
 
%% Variant subsystems for different thrust scenarios

% SCENARIO = 1; % constant thrust, constant angle
% SCENARIO = 2; % varying thrust, varying angle

CONSTANT_F_VSS = Simulink.Variant('SCENARIO==1'); % define scneario 1 as constant force input
VARYING_F_VSS = Simulink.Variant('SCENARIO==2');  % define scenario 2 as variable force input

%% Signal-Based Model MATLAB Function Parameters
prm = [planet.g;rocket.m]; % define the parameters for signal based simulink model

%% Calculate Sim Stop Time t2
% For scenario 1
fBar = thr.F/rocket.m;                               % normalized thrust having unit same as that of acceleration
x11 = 1/2*rocket.t1^2*fBar*cosd(thr.tht);            % horizontal position of rocket at burnout time
x21 = 1/2*rocket.t1^2*(fBar*sind(thr.tht)-planet.g); % vertical position of rocket at burnout time
x31 =  fBar*cosd(thr.tht)*rocket.t1;                 % horizontal speed of rocket at burnout time
x41 = (fBar*sind(thr.tht)-planet.g)*rocket.t1;       % vertical speed of rocket at burnout time
kk1 = sqrt(x41^2 + 2*planet.g*x21);                   % intermediate variable
tau1 = (x41+kk1)/planet.g;                             % time elapsed from burnout(t1) to impact(t2-1)
Impact.t1 = rocket.t1+tau1;                            % impact time in seconds
t21 = Impact.t1 + 1;                                   % Simulation stop time in seconds

% For scenario 2
fBar = thr.F/rocket.m;                               % normalized thrust having unit same as that of acceleration
thr.tht2 = thr.tht + thr.amp*cos(thr.w*rocket.t1);   % Thrust angle at burnout time
x12 = (1/2*(rocket.t1^2))*fBar*cosd(thr.tht2);        % horizontal position of rocket at burnout time
x22 = (1/2*(rocket.t1^2))*(fBar*sind(thr.tht2)-planet.g); % vertical position of rocket at burnout time
x32 =  fBar*cosd(thr.tht2)*rocket.t1;                 % horizontal speed of rocket at burnout time
x42 = (fBar*sind(thr.tht2)-planet.g)*rocket.t1;       % vertical speed of rocket at burnout time
kk2 = sqrt(x42^2 + 2*planet.g*x21);                   % intermediate variable
tau2 = (x41+kk2)/planet.g;                             % time elapsed from burnout(t1) to impact(t2-1)
Impact.t2 = rocket.t1+tau2;                            % impact time in seconds
t22 = Impact.t2 + 1;                                   % Simulation stop time in seconds

t2 = max(t21,t22);    % Take maximum of end time from both scenarios as simulation stop time.

%% Simulate Both Thrust Scenarios

SCENARIO = 1; % constant thrust and angle variant model
rocketSimOut1 = sim('rocketSim','SignalLoggingName','sdat'); % run the simulation and save data in rocketSimOut1 struct.

SCENARIO = 2; % constant thrust, time-varying angle variant model
rocketSimOut2 = sim('rocketSim','SignalLoggingName','sdat'); % run the simulation and save data in rocketSimOut2 struct.

%% Extract Data and Plot
RS = 5;  % Variable for extracting the output data of signal based model
SX = 3;  % Variable for extracting the X position data from simscape model
SY = 4;  % Variable for extracting the X position data from simscape model
VX = 1;  % Variable for extracting the X velocity data from simscape model
VY = 2;  % Variable for extracting the Y velocity data from simscape model

% Scenario 1 Results (Constant Thrust, Constant Angle)
s1.t = rocketSimOut1.sdat{RS}.Values.Time;  % Save simulation time values in s1.t array
s1.x = rocketSimOut1.sdat{RS}.Values.Data(:,1); % Save horizontal position values of signal based model in s1.x array
s1.y = rocketSimOut1.sdat{RS}.Values.Data(:,2); % Save vertical position values of signal based model in s1.y array
s1.xd = rocketSimOut1.sdat{RS}.Values.Data(:,3); % Save horizontal speed values of signal based model in s1.xd array
s1.yd = rocketSimOut1.sdat{RS}.Values.Data(:,4); % Save vertical speed values of signal based model in s1.yd array
s1.ff = rocketSimOut1.sdat{RS}.Values.Data(:,5);  % Save thrust magnitude values of signal based model in s1.ff array
s1.aa = rocketSimOut1.sdat{RS}.Values.Data(:,6)*180/pi; % Save thrust angle values of signal based model in s1.aa array
s1.sx = rocketSimOut1.sdat{SX}.Values.Data(:,1); % Save horizontal position values of simscape model in s1.sx array
s1.sy = rocketSimOut1.sdat{SY}.Values.Data(:,1); % Save vertical position values of simscape model in s1.sy array
s1.sxd = rocketSimOut1.sdat{VX}.Values.Data(:,1); % Save horizontal velocity values of simscape model in s1.sxd array
s1.syd = rocketSimOut1.sdat{VY}.Values.Data(:,1); % Save vertical velocity values of simscape model in s1.syd array
% 
% % Scenario 2 Results (Constant Thrust, Time-Varying Angle)
s2.t = rocketSimOut2.sdat{RS}.Values.Time;  % Save simulation time values in s2.t array
s2.x = rocketSimOut2.sdat{RS}.Values.Data(:,1); % Save horizontal position values of signal based model in s2.x array
s2.y = rocketSimOut2.sdat{RS}.Values.Data(:,2); % Save vertical position values of signal based model in s2.y array
s2.xd = rocketSimOut2.sdat{RS}.Values.Data(:,3); % Save horizontal speed values of signal based model in s2.xd array
s2.yd = rocketSimOut2.sdat{RS}.Values.Data(:,4);  % Save vertical speed values of signal based model in s2.yd array
s2.ff = rocketSimOut2.sdat{RS}.Values.Data(:,5); % Save thrust magnitude values of signal based model in s2.ff array
s2.aa = rocketSimOut2.sdat{RS}.Values.Data(:,6)*180/pi; % Save thrust angle values of signal based model in s2.aa array
s2.sx = rocketSimOut2.sdat{SX}.Values.Data(:,1); % Save horizontal position values of simscape model in s2.sx array
s2.sy = rocketSimOut2.sdat{SY}.Values.Data(:,1); % Save vertical position values of simscape model in s2.sy array
s2.sxd = rocketSimOut2.sdat{VX}.Values.Data(:,1); % Save horizontal velocity values of simscape model in s2.sxd array
s2.syd = rocketSimOut2.sdat{VY}.Values.Data(:,1); % Save vertical velocity values of simscape model in s2.syd array
% 
% % Extract indices at burnout and impact for both runs
s1.idBurnOut = find(s1.ff==0,1);   % Find the index at burnout for scenario 1
s1.tBurnOut  = s1.t(s1.idBurnOut); % Find the value of time at burnout index for scenario 1
s1.idImpact  = find(s1.y<0,1) - 1; % Find the index at impact for scenario 1
s1.tImpact   = s1.t(s1.idImpact);  % Find the value of time at impact index for scenario 1

s2.idBurnOut = find(s2.ff==0,1);   % Find the index at burnout for scenario 2
s2.tBurnOut  = s2.t(s2.idBurnOut); % Find the value of time at burnout index for scenario 2
s2.idImpact  = find(s2.y<0,1) - 1; % Find the index at impact for scenario 2
s2.tImpact   = s2.t(s2.idImpact);  % Find the value of time at impact index for scenario 2

%% Calculate Energy at Burnout (EO) and Impact (Ef)
s1.E0 = 1/2*rocket.m*( (s1.xd(s1.idBurnOut))^2 + (s1.yd(s1.idBurnOut))^2 ) + ...
    rocket.m*planet.g*s1.y(s1.idBurnOut);   % Energy at burnout for scenario 1
s1.Ef = 1/2*rocket.m*( (s1.xd(s1.idImpact))^2 + (s1.yd(s1.idImpact))^2 ) + ...
    rocket.m*planet.g*s1.y(s1.idImpact);    % Energy at impact for scenario 1

s2.E0 = 1/2*rocket.m*( (s2.xd(s2.idBurnOut))^2 + (s2.yd(s2.idBurnOut))^2 ) + ...
    rocket.m*planet.g*s2.y(s2.idBurnOut);   % Energy at burnout for scenario 2
s2.Ef = 1/2*rocket.m*( (s2.xd(s2.idImpact))^2 + (s2.yd(s2.idImpact))^2 ) + ...
    rocket.m*planet.g*s2.y(s2.idImpact);    % Energy at impact for scenario 2

%% Calculate Total Energy During Entire Trajectory for both scenarios
s1.E = 1/2*rocket.m*( (s1.xd).^2 + (s1.yd).^2 ) + ...
    rocket.m*planet.g*(s1.y);   % Total Energy calculated for scneario 1

s2.E = 1/2*rocket.m*( (s2.xd).^2 + (s2.yd).^2 ) + ...
    rocket.m*planet.g*(s2.y);   % Total Energy calculated for scneario 2

%% Calculate burn out errors
e.e11 = x11 - s1.x(s1.idBurnOut);  % Error in horizontal position at burnout time
e.e21 = x21 - s1.y(s1.idBurnOut);  % Error in vertical position at burnout time
e.e31 = x31 - s1.xd(s1.idBurnOut); % Error in horizontal velocity at burnout time
e.e41 = x41 - s1.yd(s1.idBurnOut); % Error in vertical velocity at burnout time
fprintf('<strong>------------------------------------------------------------------------------------------------------------------</strong>\n');
fprintf('<strong>Burnout Errors </strong>\n'); % Print the heading as burnout errors
fprintf('x  Error = %f \n',e.e11);  % Print horizontal position error
fprintf('y  Error = %f \n',e.e21);  % Print vertical position error
fprintf('Vx Error = %f \n',e.e31);  % Print horizontal velocity error
fprintf('Vy Error = %f \n',e.e41);  % Print vertical velocity error

%% Compare vertical motion for both scenarios
% plot figure 1 as time vs vertical motion of both scenarios.
figure(1);plot(s1.t(s1.idBurnOut:s1.idImpact),s1.y(s1.idBurnOut:s1.idImpact),...)
  s2.t(s2.idBurnOut:s2.idImpact),s2.y(s2.idBurnOut:s2.idImpact))
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Y (m)');       % Give Y label as Y in meters
% Define the legends
legend('Scenario = 1 = Const F and Ang','Scenario = 2 = Const F, Varying Ang');
title("Vertical motions for both scenarios"); % Give the title
fprintf('<strong>-----------------------------------------------------------------------------------------------------------------</strong>\n');
fprintf('<strong>Refer Figure 1 for comparison of vertical motion of both scenarios</strong>\n');  % Print this heading

%% Plot Energy Error from burnout to impact (should be 0)
% plot figure 2 as time vs energy error from burn out to impact
figure(2);
plot(s1.t(s1.idBurnOut:s1.idImpact),abs(s1.E(s1.idBurnOut:s1.idImpact)-s1.E0),'k');
hold on
plot(s2.t(s2.idBurnOut:s2.idImpact),abs(s2.E(s2.idBurnOut:s2.idImpact)-s2.E0),'r');
xlabel('Time (sec)');       % Give X label as Time in seconds
ylabel('Energy Error (Joules)'); % Give Y label as Energy error in joules
axis([0 10 0 4e-7]);     % specify X and Y limits
legend('Scenario 1','Scenario 2'); % Give the legends
title("Energy error from burnout to impact"); % Give the title
fprintf('<strong>-----------------------------------------------------------------------------------------------------------------</strong>\n');
fprintf('<strong>Refer Figure 2 for energy errors from burnout to impact</strong>\n');  % Print this heading

%% Verification part - Task 2a
% Generate the exact solution
FBar1.X = [fBar*cosd(thr.tht), 0];  % Horizontal component of nomralized thrust
FBar1.Y = [fBar*sind(thr.tht)-planet.g, -planet.g];  % Vertical component of nomralized thrust
i = 1;
for t1 = 0:simPrm.h:t2  % t1 goes from 0 to t2 with step of h
    if t1 <=5  % if t1 is less than or equal to 5
    exact.x1(i,:) = rocket.ics(1) + rocket.ics(3)*t1 + 1/2*t1^2*FBar1.X(1);  % define exact value of horizontal position
    exact.xd1(i,:) = rocket.ics(3) + FBar1.X(1)*t1;   % define exact value of horizontal velocity
    exact.y1(i,:) = rocket.ics(2) + rocket.ics(4)*t1 + 1/2*t1^2*FBar1.Y(1);  % define exact value of vertical position
    exact.yd1(i,:) = rocket.ics(4) + FBar1.Y(1)*t1;  % define exact value of vertical velocity
    bout = i;  % make the values at burnout equal to latest value
    else
    exact.x1(i,:) = exact.x1(bout) + (exact.xd1(bout))*(t1-rocket.t1) + 1/2*(t1-rocket.t1)^2*FBar1.X(2); % define exact value of horizontal position 
    exact.xd1(i,:) = exact.xd1(bout) + FBar1.X(2)*(t1-rocket.t1);  % define exact value of horizontal velocity
    exact.y1(i,:) = exact.y1(bout) + (exact.yd1(bout))*(t1-rocket.t1) + 1/2*(t1-rocket.t1)^2*FBar1.Y(2); % define exact value of vertical position
    exact.yd1(i,:) = exact.yd1(bout) + FBar1.Y(2)*(t1-rocket.t1);  % define exact value of vertical velocity
    exact.f1(i,:) = 0;
    end
    exact.t1(i,:) = t1;
    i = i+1;  % Repeat the loop for generating next value
end

% Calculate, print and plot errors for all states between exact solution and simulink solver
ts_error.X = abs(exact.x1(:,1)-s1.x(:,1));  % error in horizontal position between exact solution and simulink solver
ts_error.Y = abs(exact.y1(:,1)-s1.y(:,1));  % error in vertical position between exact solution and simulink solver
ts_error.Xd = abs(exact.xd1(:,1)-s1.xd(:,1));  % error in horizontal velocity between exact solution and simulink solver
ts_error.Yd = abs(exact.yd1(:,1)-s1.yd(:,1));  % error in vertical velocity between exact solution and simulink solver

figure(3);  % plot figure 3 as time vs error in exact solution vs simulation
plot(exact.t1,ts_error.X,'k');  % plot t vs error in horizontal position
hold on
plot(exact.t1,ts_error.Y,'r');  % plot t vs error in vertical position
plot(exact.t1,ts_error.Xd,'b');  % plot t vs error in vertical position
plot(exact.t1,ts_error.Yd,'m');  % plot t vs error in vertical position
axis([0 10 0 5e-10]);
legend('X-error','Y-error','Vx-error','Vy-error'); % Give the legends
title({                                            % Give the title
     ('Error in values between') 
      ('exact solution and simulink solver')  
      }); 
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('error (m or m/s)');  % Give Y label as error

fprintf('<strong>-----------------------------------------------------------------------------------------------------------------</strong>\n');
fprintf('<strong>Verification part 1 - Erros between exact solution and simulink solver; Refer Figure 3</strong>\n');  % Print this heading
fprintf('Max X error for exact solution vs simulink solver = %f \n',max(ts_error.X)); % Print maximum of error in X for exact vs simulink
fprintf('Max Y error for exact solution vs simulink solver = %f \n',max(ts_error.Y)); % Print maximum of error in Y for exact vs simulink
fprintf('Max Vx error for exact solution vs simulink solver = %f \n',max(ts_error.Xd)); % Print maximum of error in Xd for exact vs simulink
fprintf('Max Vy error for exact solution vs simulink solver = %f \n',max(ts_error.Yd)); % Print maximum of error in Yd for exact vs simulink

if max(ts_error.X) < 10e-6  && max(ts_error.Y) < 10e-6  && max(ts_error.Xd) < 10e-6 && max(ts_error.Yd) < 10e-6
    fprintf('<strong>Simulated and closed solution results are matching</strong>\n');
    fprintf('<strong>Integration method and time step is appropriate</strong>\n');
else
    fprintf('<strong>Simulated and closed solution results are not matching</strong>\n');
    fprintf('<strong>Integration method and time step is not appropriate</strong>\n');
end

%% Verification part - Task 2b
% Calculate, print and plot errors for all signals between simscape multibody and signal based simulink model
sc_error.X = abs(s2.sx(:,1)-s2.x(:,1));  % error in horizontal position between simscape and signal based
sc_error.Y = abs(s2.sy(:,1)-s2.y(:,1));  % error in vertical position between simscape and signal based
sc_error.Xd = abs(s2.sxd(:,1)-s2.xd(:,1));  % error in horizontal velocity between simscape and signal based
sc_error.Yd = abs(s2.syd(:,1)-s2.yd(:,1));  % error in vertical velocity between simscape and signal based

figure(4);  % plot figure 4 as time vs error in simscape vs signal based simulink model
plot(s2.t,s2.x(:,1),'k'); % X trajectory for signal based model
hold on
plot(s2.t,s2.sx(:,1),'r'); % X trajectory for simscape based model
plot(s2.t,s2.y(:,1),'b'); % Y trajectory for signal based model
plot(s2.t,s2.sy(:,1),'m'); % Y trajectory for simscape based model
axis([0 Impact.t2 -100 500]);  % Give the limits of both axis
legend('X-signal based','X-Comppnent based','Y-signal based','Y-Comppnent based'); % Give the legends
title("X & Y trajectories for signal based and component based models"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('X & Y (m)');  % Give Y label as X & Y

fprintf('<strong>-----------------------------------------------------------------------------------------------------------------</strong>\n');
% Print this heading
fprintf('<strong>Verification part 2 - Identical trajectory for simscape multibody and signal based simulink model; Refer Figure 4</strong>\n');
fprintf('Max X error for component based vs signal based model = %f \n',max(sc_error.X)); % Print maximum of error in X for simscape vs simulink
fprintf('Max Y error for component based vs signal based model = %f \n',max(sc_error.Y)); % Print maximum of error in Y for simscape vs simulink
fprintf('Max Vx error for component based vs signal based model = %f \n',max(sc_error.Xd)); % Print maximum of error in Xd for simscape vs simulink
fprintf('Max Vy error for component based vs signal based model = %f \n',max(sc_error.Yd)); % Print maximum of error in Yd for simscape vs simulink

if max(sc_error.X) < 10e-6  && max(sc_error.Y) < 10e-6  && max(sc_error.Xd) < 10e-6 && max(sc_error.Yd) < 10e-6
    fprintf('<strong>Both models trajectories are identical so both models are implemented correctly; Refer figure 4</strong>\n');
else
    fprintf('<strong>Both models trajectories are not identical</strong>\n');
end

%% Verification part - Task 2c
% Calculate & print change in energy from burnout to impact and plot total energy vs time to prove that energy is conserved
energychange.s1 = abs(s1.E(s1.idBurnOut)-s1.E(s1.idImpact));   % Calculate change in total energy for scenario 1
energychange.s2 = abs(s2.E(s2.idBurnOut)-s2.E(s2.idImpact));  % Calculate change in total energy for scenario 2
fprintf('<strong>------------------------------------------------------------------------------------------------------------------</strong>\n');
fprintf('<strong>Verification part 3 - Energy Conservation from burnout to impact; Refer Figure 5</strong>\n');  % Print this heading
fprintf('Change in total energy from burnout to impact for scenario 1 = %f \n',energychange.s1); % Print change in total energy for scenario 1
fprintf('Change in total energy from burnout to impact for scenario 2 = %f \n',energychange.s2); % Print change in total energy for scenario 2

% Plot the total energy vs time from burn out to impact to show that energy
% changes by very very small amount which confirms that energy is conserved
figure(5);  % plot figure 5 as time vs energy and energy errors
plot(s1.t(s1.idBurnOut:s1.idImpact),s1.E(s1.idBurnOut:s1.idImpact),'k'); % error in horizontal position between simscape and signal based
hold on
plot(s2.t(s2.idBurnOut:s2.idImpact),s2.E(s2.idBurnOut:s2.idImpact),'r'); % error in vertical position between simscape and signal based
axis([rocket.t1 Impact.t2 1.1e6 1.3e6]);
legend('Total energy for scenario 1','Total energy for scenario 2'); % Give the legends
title("Verification of energy conservation"); % Give the title
xlabel('Time (sec)');  % Give X label as Time in seconds
ylabel('Energy (Joules)');  % Give Y label as error
if max(energychange.s1) < 10e-6  && max(energychange.s2) < 10e-6 
    fprintf('<strong>Total energy is constant from burnout to impact thus it is conserved; Refer figure 5 & figure 2</strong>\n');
else
    fprintf('<strong>Total energy is changing from burnout to impact</strong>\n');
end
%% END
