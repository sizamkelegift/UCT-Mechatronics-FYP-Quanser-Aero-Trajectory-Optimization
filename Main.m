%MAIN.m  --  solve problem for Quanset aero platform
%
%
%

clc; clear;
addpath ../../

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Parameters for the dynamics function                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
dyn.mb = 1.15;  % body mass
dyn.Dt = 0.158;
dyn.Dm = 0.35;
dyn.Dz_ = 0.0211;
dyn.Dy_ = 0.0226;
dyn.Ksp_ = 0.0375;
dyn.Iz_ = 0.022;
dyn.Iy_ = 0.0219;

t0 = 0;
tF = 4.0;               %For now, force it to take exactly this much time.
x0 = [0;0];              %initial angles   
xF = [pi/10;pi/2];       %final angles 
dx0 = [0;0];             %initial angle rates
dxF = [0;0];             %final angle rates
maxTorque = 24;           % Max torque

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,u)( aeroDynamics(x,u,dyn) );

problem.func.pathObj = @(t,x,u)( sum(u.^2,1) );  %Simple torque-squared

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = t0;
problem.bounds.initialTime.upp = t0;
problem.bounds.finalTime.low = tF;
problem.bounds.finalTime.upp = tF;

% State: [q1;q2;dq1;dq2];

problem.bounds.state.low = [-7*pi/18; -2*pi; -inf(2,1)];
problem.bounds.state.upp = [ 7*pi/18;  2*pi;  inf(2,1)];

problem.bounds.initialState.low = [x0; dx0];
problem.bounds.initialState.upp = [x0; dx0];
problem.bounds.finalState.low = [xF; dxF];
problem.bounds.finalState.upp = [xF; dxF];

problem.bounds.control.low = -maxTorque*[1;1];
problem.bounds.control.upp = maxTorque*[1;1];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Options:                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options(1).method = 'trapezoid'; %  <-- this is robust, but less accurate
%problem.options(1).trapezoid.nGrid = 20;
%problem.options(2).method = 'trapezoid';
%problem.options(2).trapezoid.nGrid = 40;
%problem.options(3).method = 'trapezoid';
problem.options(3).trapezoid.nGrid = 60;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [t0, tF];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = zeros(2,2);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Solve!                                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);

% Interpolate the solution on a uniform grid for plotting and animation:
tGrid = soln(end).grid.time;
t = linspace(tGrid(1),tGrid(end),100);
z = soln(end).interp.state(t);
u = soln(end).interp.control(t);

%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%HINT:  type help animate to figure out how to use the keyboard to interact
%with the animation (slow motion, pause, jump forward / backward...)
%%
Dt = 0.158;
Dm = 0.35;

psi= z(2,:);
tht= z(1,:);

p1 = [Dt*cos(tht).*cos(psi);Dt*cos(tht).*sin(psi);Dm + Dm*sin(tht)];
p2 = [-Dt*cos(tht).*cos(psi);-Dt*cos(tht).*sin(psi);Dm - Dm*sin(tht)];
%%

figure(3); clf;
nFrame = 9;  %Number of frames to draw
drawStopActionAero(t,p1,p2,nFrame);

%%

% Animate the results:
A.plotFunc = @(t,z)( drawAero(t,z,dyn) );
A.speed = 0.25;
A.figNum = 101;
animate(t,z,A)

%%
% Plot the results:
figure(2); clf; plotAero(t,z,u);

% Draw a stop-action animation:
% figure(3); clf; drawStopActionAero(soln(end),dyn);