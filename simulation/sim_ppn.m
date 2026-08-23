clear;
clc;
close all;

%% =========================================================
%  3D INTERCEPTOR UAV - PPN INTERCEPTION SIMULATION
%
%  Flight sequence:
%
%  1. UAV starts pointed directly toward target
%  2. Direct-to-target flight
%  3. Smooth transition into PPN
%  4. PPN pursuit
%  5. Interception
%
%  REVISION NOTES (this version):
%   - UAV / target 3D bodies can now be loaded from real STL files
%     instead of (or in addition to) the hand-built patch geometry.
%   - readSTL() is a toolbox-free binary+ASCII STL parser, so it does
%     not depend on which "stlread" (if any) happens to be on your
%     MATLAB path.
%   - If STL_UAV_FILE / STL_TARGET_FILE below are empty, missing, or
%     fail to load for any reason, the script automatically falls
%     back to the original procedural (patch-built) models, so this
%     file always runs even with no STL files at hand.
%   - A camlight + gouraud lighting was added so STL meshes shade
%     properly (flat patches are otherwise unlit/black).
%   - Visual polish pass: legend no longer floats away from the 3D
%     box, the flight-phase label is folded into the title (so it
%     can never overlap another text object), the four telemetry
%     graphs have more breathing room between them, and the target
%     model is colored orange.
%   - TARGET NOW MANEUVERS: instead of flying a dead-straight line,
%     the target performs a smooth, repeating heading zigzag (an
%     S-turn) plus a slower altitude weave, plus a bit of small
%     high-frequency jitter/buffet layered on top. Speed magnitude
%     is held constant; the previously-unused shakeAmp/shakeFreq
%     variables are now actually wired in for that jitter. The
%     jitter is recomputed fresh from the current time every step
%     (never integrated), so it can never drift or accumulate error.
% ==========================================================


%% =========================================================
% STL MODEL CONFIGURATION
% ==========================================================

% Set these to '' (empty) to use the built-in procedural models,
% or to a path to a .stl file to render that mesh instead.
STL_UAV_FILE    = 'uav_model.stl';       % interceptor UAV mesh
STL_TARGET_FILE = 'target_model.stl';    % target aircraft mesh

% Desired nose-to-tail length of each STL model AFTER auto-scaling,
% in the same units as the simulation (meters). Tune to taste.
STL_UAV_LENGTH    = 30;
STL_TARGET_LENGTH = 220;

% Orientation fix: your animation always rotates the model's LOCAL
% +X axis to point along the velocity vector. Most STL exports are
% NOT authored nose-along-+X, so you'll typically need to supply a
% fixed rotation here.
STL_UAV_ROTFIX    = [0 0 1;0 1 0;-1 0 0];
STL_TARGET_ROTFIX = [0 -1 0;1 0 0;0 0 1];

% Colors used only for the STL mesh (procedural fallback keeps its
% own hard-coded colors, updated below to match)
STL_UAV_COLOR    = [0.15 0.15 0.18];
STL_TARGET_COLOR = [0.95 0.45 0.05];      % orange


%% =========================================================
% SIMULATION PARAMETERS
% ==========================================================

dt   = 0.002;          % Simulation time step [s]
tMax = 40;             % Maximum simulation time [s]

% Animation update interval
% 1 = update every simulation step
% 5 = update every 5 simulation steps
% 10 = update every 10 simulation steps
animationStep = 10;



N = 4;                 % Navigation constant

interceptRadius = 5;  % Intercept distance [m]


%% =========================================================
% INITIAL GUIDANCE PARAMETERS
% ==========================================================

% Duration of initial direct-to-target flight
initialGuidanceTime = 1.5;        % [s]

% End of transition into PPN
transitionEnd = 3.5;              % [s]

% Initial UAV speed
initialSpeed = 280;                % [m/s]

% Maximum steering acceleration during
% initial direct-to-target phase
transitionAcceleration = 120;     % [m/s^2]

% If true, the UAV behaves like an "ideal missile": commanded
% acceleration only changes its HEADING, never its speed.
maintainConstantSpeed = true;


%% =========================================================
% TARGET INITIAL CONDITIONS
% ==========================================================

% Target shake parameters (small, high-frequency positional buffet -
% now actually used, see "TARGET ZIGZAG + JITTER" inside the loop)
shakeAmp   = [20; 20; 20];     % Shake amplitude in [X; Y; Z] (meters)
shakeFreq  = [1; 1; 1];     % Shake frequency in [X; Y; Z] (rad/s)
rT = [2000; 0; 1000];          % Target position [m]

VT = [-100; 0; 0];                % Target velocity [m/s]


%% =========================================================
% TARGET ZIGZAG MANEUVER PARAMETERS
% ==========================================================
% The target no longer flies a dead-straight line. Every step its
% heading is swept left/right through a smooth, repeating triangle
% wave (a genuine zigzag - straight-line legs joined at an angle,
% not a lazy sine curve), plus a slower/smaller altitude weave so it
% doesn't look like it's confined to a flat plane. Ground speed is
% held constant throughout; only the heading direction changes.

zigzagYawAmp      = 0.1;     % [deg] max heading deviation, left/right
zigzagYawPeriod   = 10;      % [s]   time for one full left-right-left cycle

zigzagPitchAmp    = 1;      % [deg] max altitude-weave deviation
zigzagPitchPeriod = 30;     % [s]   intentionally not a multiple of
                            %       zigzagYawPeriod, so the two
                            %       weaves drift in and out of phase
                            %       instead of repeating in lockstep

targetSpeed   = norm(VT);          % target holds this speed throughout
targetBaseDir = VT / targetSpeed;  % nominal (unmaneuvered) heading

% Build a stable local right/up frame around the base heading so the
% heading can be rotated left/right and up/down each step.

worldUp = [0;0;1];

if abs(dot(targetBaseDir,worldUp)) > 0.98

    worldUp = [0;1;0];

end

targetRightDir = cross(targetBaseDir,worldUp);
targetRightDir = targetRightDir / norm(targetRightDir);

targetUpDir = cross(targetRightDir,targetBaseDir);
targetUpDir = targetUpDir / norm(targetUpDir);

% Fixed phase offsets for the small XYZ jitter so all three axes
% don't happen to cross zero at the same instant
shakePhase = [0; 1.1; 2.3];


%% =========================================================
% UAV INITIAL CONDITIONS
% ==========================================================

rM = [0; 0; 0];               % UAV launch position [m]


%% =========================================================
% INITIAL UAV HEADING
% ==========================================================

% Calculate initial line-of-sight vector
initialLOS = rT - rM;

% Normalize it
initialLOS = initialLOS / norm(initialLOS);

% UAV starts moving directly toward target
VM = initialLOS * initialSpeed;


%% =========================================================
% TIME VECTOR
% ==========================================================

time = 0:dt:tMax;

numSteps = length(time);


%% =========================================================
% STORAGE
% ==========================================================

uavPos = zeros(3,numSteps);

targetPos = zeros(3,numSteps);

uavVel = zeros(3,numSteps);

targetVel = zeros(3,numSteps);

rangeHist = zeros(1,numSteps);

VcHist = zeros(1,numSteps);

omegaHist = zeros(1,numSteps);

accelHist = zeros(1,numSteps);

phaseHist = strings(1,numSteps);


%% =========================================================
% SIMULATION
% ==========================================================

interceptIndex = numSteps;


for k = 1:numSteps

    %% =====================================================
    % CURRENT TIME
    % ======================================================

    t = time(k);


    %% =====================================================
    % TARGET ZIGZAG + JITTER (recomputed every step from t, never
    % integrated, so it can never drift or accumulate error)
    % ======================================================

    % Smooth triangle waves (piecewise-linear ramps, not sinusoidal
    % S-curves) so the turns look like genuine banked heading
    % changes rather than a lazy sine wander.

    yawTriangle   = (2/pi) * asin(sin(2*pi*t/zigzagYawPeriod));

    pitchTriangle = (2/pi) * asin(sin(2*pi*t/zigzagPitchPeriod + pi/3));


    yawAngle   = deg2rad(zigzagYawAmp)   * yawTriangle;

    pitchAngle = deg2rad(zigzagPitchAmp) * pitchTriangle;


    % Rotate the base heading by yaw (about targetUpDir), then by
    % pitch (about targetRightDir)

    dirYawed = cos(yawAngle)*targetBaseDir + sin(yawAngle)*targetRightDir;

    dirYawed = dirYawed / norm(dirYawed);


    targetDirNow = cos(pitchAngle)*dirYawed + sin(pitchAngle)*targetUpDir;

    targetDirNow = targetDirNow / norm(targetDirNow);


    VT = targetSpeed * targetDirNow;


    % Small high-frequency positional jitter (buffet/turbulence),
    % layered on top of the smooth zigzag path for the displayed and
    % guided-against position only - it never feeds back into the
    % position integrator, so it cannot accumulate.

    shakeOffset = shakeAmp .* sin(shakeFreq*t + shakePhase);

    trueTargetPos = rT + shakeOffset;


    %% =====================================================
    % STORE CURRENT STATE
    % ======================================================

    uavPos(:,k) = rM;

    targetPos(:,k) = trueTargetPos;

    uavVel(:,k) = VM;

    targetVel(:,k) = VT;


    %% =====================================================
    % RELATIVE POSITION
    % ======================================================

    r = trueTargetPos - rM;

    R = norm(r);

    rangeHist(k) = R;


    %% =====================================================
    % RELATIVE VELOCITY
    % ======================================================

    Vrel = VT - VM;


    %% =====================================================
    % CLOSING VELOCITY
    % ======================================================

    if R > 1e-9

        Vc = -dot(r,Vrel) / R;

    else

        Vc = 0;

    end

    VcHist(k) = Vc;


    %% =====================================================
    % LOS ANGULAR VELOCITY
    % ======================================================

    if R > 1e-9

        omega = cross(r,Vrel) / R^2;

        omegaMag = norm(omega);

    else

        omega = [0;0;0];

        omegaMag = 0;

    end

    omegaHist(k) = omegaMag;


    %% =====================================================
    % UAV VELOCITY MAGNITUDE
    % ======================================================

    VMmag = norm(VM);


    %% =====================================================
    % UAV CURRENT DIRECTION
    % ======================================================

    if VMmag > 1e-6

        VHat = VM / VMmag;

    else

        VHat = initialLOS;

    end


    %% =====================================================
    % PURE PPN ACCELERATION
    % ======================================================

    aPPN = N * Vc * cross(omega,VHat);

    ppnAcceleration = norm(aPPN);


    %% =====================================================
    % FLIGHT CONTROL
    % ======================================================

    if t < initialGuidanceTime

        %% =================================================
        % PHASE 1
        % DIRECT-TO-TARGET
        % ==================================================

        phaseHist(k) = "DIRECT TO TARGET";


        % Current line-of-sight direction

        if R > 1e-9

            desiredDirection = r / R;

        else

            desiredDirection = initialLOS;

        end


        % Current UAV direction

        if VMmag > 1e-6

            currentDirection = VM / VMmag;

        else

            currentDirection = initialLOS;

        end


        % Direction error

        directionError = ...
            cross(currentDirection,desiredDirection);


        % Steering acceleration

        aCommand = ...
            transitionAcceleration * directionError;


    elseif t < transitionEnd

        %% =================================================
        % PHASE 2
        % TRANSITION TO PPN
        % ==================================================

        phaseHist(k) = "PPN TRANSITION";


        % Transition factor

        alpha = ...
            (t - initialGuidanceTime) / ...
            (transitionEnd - initialGuidanceTime);

        alpha = max(0,min(1,alpha));


        % Current LOS direction

        if R > 1e-9

            losHat = r / R;

        else

            losHat = initialLOS;

        end


        % Current UAV direction

        if VMmag > 1e-6

            currentDirection = VM / VMmag;

        else

            currentDirection = losHat;

        end


        % Direction error

        directionError = ...
            cross(currentDirection,losHat);


        % Direct-to-target steering

        aDirect = ...
            transitionAcceleration * directionError;


        % Smoothly blend direct guidance
        % into PPN guidance

        aCommand = ...
            (1-alpha)*aDirect + ...
            alpha*aPPN;


    else

        %% =================================================
        % PHASE 3
        % PURE PPN PURSUIT
        % ==================================================

        phaseHist(k) = "PPN PURSUIT";

        aCommand = aPPN;

    end


    %% =====================================================
    % STORE COMMAND
    % ======================================================

    accelHist(k) = norm(aCommand);


    %% =====================================================
    % INTERCEPT CHECK
    % ======================================================

    if R < interceptRadius

        interceptIndex = k;


        fprintf('\n');
        fprintf('=================================================\n');
        fprintf('          INTERCEPTOR UAV SIMULATION\n');
        fprintf('=================================================\n');
        fprintf('Flight Phase       : %s\n',phaseHist(k));
        fprintf('Navigation Constant: N = %.1f\n',N);
        fprintf('Intercept Time     : %.3f s\n',t);
        fprintf('Final Range        : %.3f m\n',R);
        fprintf('Closing Velocity   : %.3f m/s\n',Vc);
        fprintf('LOS Angular Rate   : %.6f rad/s\n',omegaMag);
        fprintf('Final Acceleration : %.3f m/s^2\n',norm(aCommand));
        fprintf('Final UAV Speed    : %.3f m/s\n',VMmag);
        fprintf('=================================================\n');

        break;

    end


    %% =====================================================
    % UPDATE UAV VELOCITY
    % ======================================================

    VM = VM + aCommand * dt;


    %% =====================================================
    % ENFORCE CONSTANT SPEED (ideal-missile assumption)
    % ======================================================

    if maintainConstantSpeed

        VMnorm = norm(VM);

        if VMnorm > 1e-6

            VM = VM / VMnorm * initialSpeed;

        end

    end


    %% =====================================================
    % UPDATE UAV POSITION
    % ======================================================

    rM = rM + VM * dt;


    %% =====================================================
    % UPDATE TARGET POSITION (smooth zigzag base path only - the
    % small jitter above is recomputed fresh every step and is
    % intentionally NOT integrated here, so it can never drift)
    % ======================================================

    rT = rT + VT * dt;

end


%% =========================================================
% TRIM RESULTS
% ==========================================================

time = time(1:interceptIndex);

uavPos = uavPos(:,1:interceptIndex);

targetPos = targetPos(:,1:interceptIndex);

uavVel = uavVel(:,1:interceptIndex);

targetVel = targetVel(:,1:interceptIndex);

rangeHist = rangeHist(1:interceptIndex);

VcHist = VcHist(1:interceptIndex);

omegaHist = omegaHist(1:interceptIndex);

accelHist = accelHist(1:interceptIndex);

phaseHist = phaseHist(1:interceptIndex);

numSteps = length(time);


%% =========================================================
% CREATE MAIN FIGURE
% ==========================================================

fig = figure( ...
    'Name','3D Interceptor UAV - PPN Simulation', ...
    'Color','w', ...
    'Position',[40 40 1500 880]);


%% =========================================================
% 3D AXES
% ==========================================================

ax3D = axes( ...
    'Parent',fig, ...
    'Position',[0.04 0.08 0.60 0.84], ...
    'FontSize',10, ...
    'GridAlpha',0.25, ...
    'LineWidth',1);

hold(ax3D,'on');

grid(ax3D,'on');

box(ax3D,'on');

axis(ax3D,'equal');

view(ax3D,3);

xlabel(ax3D,'X [m]','FontSize',12,'FontWeight','bold','Color','#3270C9');
ylabel(ax3D,'Y [m]','FontSize',12,'FontWeight','bold','Color','#3270C9');
zlabel(ax3D,'Altitude Z [m]','FontSize',12,'FontWeight','bold','Color','#3270C9');

% Lighting so STL meshes shade properly (flat patches are unlit
% otherwise). Harmless for the procedural fallback models too.
camlight(ax3D,'headlight');
lighting(ax3D,'gouraud');
material(ax3D,'dull');


%% =========================================================
% AXIS LIMITS
% ==========================================================

allX = [uavPos(1,:) targetPos(1,:)];

allY = [uavPos(2,:) targetPos(2,:)];

allZ = [uavPos(3,:) targetPos(3,:)];

xMin = min(allX);
xMax = max(allX);

yMin = min(allY);
yMax = max(allY);

zMin = min(allZ);
zMax = max(allZ);


xMargin = max(250,0.05*(xMax-xMin));

yMargin = max(250,0.05*(yMax-yMin));

zMargin = max(150,0.10*(zMax-zMin));


xlim(ax3D,[xMin-xMargin xMax+xMargin]);

ylim(ax3D,[yMin-yMargin yMax+yMargin]);

zlim(ax3D,[max(0,zMin-zMargin) zMax+zMargin]);


%% =========================================================
% TRAJECTORIES
% ==========================================================

hUAVTrail = plot3( ...
    ax3D, ...
    NaN,NaN,NaN, ...
    '--', ...
    'Color', [0.00, 0.20, 0.40], ...
    'LineWidth',0.8);

hTargetTrail = plot3( ...
    ax3D, ...
    NaN,NaN,NaN, ...
    '--', ...
    'Color',[0.95 0.45 0.05], ...
    'LineWidth',0.8);


%% =========================================================
% CREATE UAV MODEL  (STL if available, else procedural fallback)
% ==========================================================

uavTransform = hgtransform( ...
    'Parent',ax3D);

uavModelLoaded = false;

if ~isempty(STL_UAV_FILE) && isfile(STL_UAV_FILE)

    try

        createSTLModel( ...
            uavTransform, ...
            STL_UAV_FILE, ...
            STL_UAV_LENGTH, ...
            STL_UAV_COLOR, ...
            STL_UAV_ROTFIX);

        uavModelLoaded = true;

        fprintf('UAV model    : loaded from %s\n',STL_UAV_FILE);

    catch stlErr

        warning('Failed to load UAV STL (%s). Falling back to procedural model.\nReason: %s', ...
            STL_UAV_FILE, stlErr.message);

    end

end

if ~uavModelLoaded

    createUAVModel( ...
        uavTransform, ...
        70);

    fprintf('UAV model    : procedural (no STL loaded)\n');

end


%% =========================================================
% CREATE TARGET AIRCRAFT  (STL if available, else procedural fallback)
% ==========================================================

targetTransform = hgtransform( ...
    'Parent',ax3D);

targetModelLoaded = false;

if ~isempty(STL_TARGET_FILE) && isfile(STL_TARGET_FILE)

    try

        createSTLModel( ...
            targetTransform, ...
            STL_TARGET_FILE, ...
            STL_TARGET_LENGTH, ...
            STL_TARGET_COLOR, ...
            STL_TARGET_ROTFIX);

        targetModelLoaded = true;

        fprintf('Target model : loaded from %s\n',STL_TARGET_FILE);

    catch stlErr

        warning('Failed to load target STL (%s). Falling back to procedural model.\nReason: %s', ...
            STL_TARGET_FILE, stlErr.message);

    end

end

if ~targetModelLoaded

    createTargetAircraft( ...
        targetTransform, ...
        100);

    fprintf('Target model : procedural (no STL loaded)\n');

end


%% =========================================================
% LAUNCH POINT
% ==========================================================

plot3( ...
    ax3D, ...
    uavPos(1,1),uavPos(2,1),uavPos(3,1), ...
    '-', ...
    'MarkerFaceColor','k', ...
    'MarkerSize',6);


%% =========================================================
% LEGEND
% ==========================================================
% NOTE: with 'axis equal' on a 3D view, MATLAB frequently shrinks the
% axes' actual drawn box to preserve aspect ratio, leaving blank
% space inside the nominal axes Position. A 'Location' such as
% 'northeast' anchors to that nominal box and can end up visually far
% from the plotted cube. Anchoring the legend to an explicit
% figure-normalized Position avoids that entirely.

lgd = legend(ax3D, ...
    [hUAVTrail hTargetTrail], ...
    {'   Interceptor UAV Trajectory', ...
     '   Target Trajectory'}, ...
    'FontSize',9);

set(lgd, ...
    'Units','normalized', ...
    'Position',[0.115 0.855 0.087 0.055], ...
    'Box','on', ...
    'Color',[0.95 0.95 0.95]);     % gray


%% =========================================================
% RIGHT-COLUMN SECTION LABEL
% ==========================================================
%{
annotation(fig,'textbox',[0.69 0.955 0.27 0.035], ...
    'String','Guidance Telemetry', ...
    'FontSize',12, ...
    'FontWeight','bold', ...
    'HorizontalAlignment','center', ...
    'EdgeColor','none');
%}

%% =========================================================
% RANGE GRAPH
% ==========================================================

ax1 = axes( ...
    'Parent',fig, ...
    'Position',[0.69 0.805 0.27 0.165], ...
    'FontSize',9, ...
    'GridAlpha',0.25);

hold(ax1,'on');

grid(ax1,'on');

box(ax1,'on');


hRange = plot( ...
    ax1,NaN,NaN, ...
    'LineWidth',1.5);


hRangeCurrent = plot( ...
    ax1,NaN,NaN, ...
    '-', ...
    'MarkerFaceColor','b');


hRangeTime = xline( ...
    ax1,0, ...
    '--', ...
    'LineWidth',1);


xlabel(ax1,'Time [s]','FontSize',9);

ylabel(ax1,'Range [m]','FontSize',9);

title(ax1,'Range R(t)','FontSize',10,'FontWeight','bold','Color','#3270C9');

xlim(ax1,[0 time(end)]);

ylim(ax1,[0 max(rangeHist)*1.05]);


%% =========================================================
% CLOSING VELOCITY GRAPH
% ==========================================================

ax2 = axes( ...
    'Parent',fig, ...
    'Position',[0.69 0.565 0.27 0.165], ...
    'FontSize',9, ...
    'GridAlpha',0.25);

hold(ax2,'on');

grid(ax2,'on');

box(ax2,'on');


hVc = plot( ...
    ax2,NaN,NaN, ...
    'LineWidth',1.5);


hVcCurrent = plot( ...
    ax2,NaN,NaN, ...
    '-', ...
    'MarkerFaceColor','g');


hVcTime = xline( ...
    ax2,0, ...
    '--', ...
    'LineWidth',1);


xlabel(ax2,'Time [s]','FontSize',9);

ylabel(ax2,'V_c [m/s]','FontSize',9);

title(ax2,'Closing Velocity','FontSize',10,'FontWeight','bold','Color','#3270C9');

xlim(ax2,[0 time(end)]);


VcMin = min(VcHist);

VcMax = max(VcHist);


if VcMin == VcMax

    VcMin = VcMin - 1;

    VcMax = VcMax + 1;

end


ylim(ax2,[VcMin*0.95 VcMax*1.05]);


%% =========================================================
% LOS RATE GRAPH
% ==========================================================

ax3 = axes( ...
    'Parent',fig, ...
    'Position',[0.69 0.325 0.27 0.165], ...
    'FontSize',9, ...
    'GridAlpha',0.25);

hold(ax3,'on');

grid(ax3,'on');

box(ax3,'on');


hOmega = plot( ...
    ax3,NaN,NaN, ...
    'LineWidth',1.5);


hOmegaCurrent = plot( ...
    ax3,NaN,NaN, ...
    '-', ...
    'MarkerFaceColor','m');


hOmegaTime = xline( ...
    ax3,0, ...
    '--', ...
    'LineWidth',1);


xlabel(ax3,'Time [s]','FontSize',9);

ylabel(ax3,'|\Omega| [rad/s]','FontSize',9);

title(ax3,'LOS Angular Rate','FontSize',10,'FontWeight','bold','Color','#3270C9');

xlim(ax3,[0 time(end)]);


omegaMax = max(omegaHist);


if omegaMax < 1e-6

    omegaMax = 1;

end


ylim(ax3,[0 omegaMax*1.1]);


%% =========================================================
% ACCELERATION GRAPH
% ==========================================================

ax4 = axes( ...
    'Parent',fig, ...
    'Position',[0.69 0.075 0.27 0.165], ...
    'FontSize',9, ...
    'GridAlpha',0.25);

hold(ax4,'on');

grid(ax4,'on');

box(ax4,'on');


hAccel = plot( ...
    ax4,NaN,NaN, ...
    'LineWidth',1.5);


hAccelCurrent = plot( ...
    ax4,NaN,NaN, ...
    '-', ...
    'MarkerFaceColor','g');


hAccelTime = xline( ...
    ax4,0, ...
    '--', ...
    'LineWidth',1);


xlabel(ax4,'Time [s]','FontSize',9);

ylabel(ax4,'Acceleration [m/s^2]','FontSize',9);

title(ax4,'Commanded Acceleration','FontSize',10,'FontWeight','bold','Color','#3270C9');

xlim(ax4,[0 time(end)]);


accelMax = max(accelHist);


if accelMax < 1

    accelMax = 1;

end


ylim(ax4,[0 accelMax*1.1]);


%% =========================================================
% ANIMATION
% ==========================================================

for k = 1:animationStep:numSteps

    %% =====================================================
    % CURRENT TIME
    % ======================================================

    tNow = time(k);


    %% =====================================================
    % CURRENT UAV STATE
    % ======================================================

    currentUAVPosition = uavPos(:,k);

    currentUAVVelocity = uavVel(:,k);


    %% =====================================================
    % CURRENT TARGET STATE
    % ======================================================

    currentTargetPosition = targetPos(:,k);

    currentTargetVelocity = targetVel(:,k);


    %% =====================================================
    % UPDATE UAV TRAJECTORY
    % ======================================================

    set(hUAVTrail, ...
        'XData',uavPos(1,1:k), ...
        'YData',uavPos(2,1:k), ...
        'ZData',uavPos(3,1:k));


    %% =====================================================
    % UPDATE TARGET TRAJECTORY
    % ======================================================

    set(hTargetTrail, ...
        'XData',targetPos(1,1:k), ...
        'YData',targetPos(2,1:k), ...
        'ZData',targetPos(3,1:k));


    %% =====================================================
    % UAV ORIENTATION
    % ======================================================

    currentSpeed = norm(currentUAVVelocity);


    if currentSpeed > 1e-6

        uavDirection = ...
            currentUAVVelocity / currentSpeed;

    else

        uavDirection = initialLOS;

    end


    %% -----------------------------------------------------
    % Rotate UAV body x-axis into velocity direction
    % ------------------------------------------------------

    Ruav = directionRotation( ...
        [1;0;0], ...
        uavDirection);


    Tuav = eye(4);

    Tuav(1:3,1:3) = Ruav;

    Tuav(1:3,4) = currentUAVPosition;


    set(uavTransform, ...
        'Matrix',Tuav);


    %% =====================================================
    % TARGET ORIENTATION (now follows the zigzag heading at each
    % frame, instead of a fixed constant direction)
    % ======================================================

    targetSpeedNow = norm(currentTargetVelocity);

    if targetSpeedNow > 1e-6

        targetDirection = currentTargetVelocity / targetSpeedNow;

    else

        targetDirection = targetBaseDir;

    end


    Rtarget = directionRotation( ...
        [1;0;0], ...
        targetDirection);


    Ttarget = eye(4);

    Ttarget(1:3,1:3) = Rtarget;

    Ttarget(1:3,4) = currentTargetPosition;


    set(targetTransform, ...
        'Matrix',Ttarget);


    %% =====================================================
    % RANGE GRAPH
    % ======================================================

    set(hRange, ...
        'XData',time(1:k), ...
        'YData',rangeHist(1:k));


    set(hRangeCurrent, ...
        'XData',tNow, ...
        'YData',rangeHist(k));


    set(hRangeTime, ...
        'Value',tNow);


    %% =====================================================
    % CLOSING VELOCITY GRAPH
    % ======================================================

    set(hVc, ...
        'XData',time(1:k), ...
        'YData',VcHist(1:k));


    set(hVcCurrent, ...
        'XData',tNow, ...
        'YData',VcHist(k));


    set(hVcTime, ...
        'Value',tNow);


    %% =====================================================
    % LOS RATE GRAPH
    % ======================================================

    set(hOmega, ...
        'XData',time(1:k), ...
        'YData',omegaHist(1:k));


    set(hOmegaCurrent, ...
        'XData',tNow, ...
        'YData',omegaHist(k));


    set(hOmegaTime, ...
        'Value',tNow);


    %% =====================================================
    % ACCELERATION GRAPH
    % ======================================================

    set(hAccel, ...
        'XData',time(1:k), ...
        'YData',accelHist(1:k));


    set(hAccelCurrent, ...
        'XData',tNow, ...
        'YData',accelHist(k));


    set(hAccelTime, ...
        'Value',tNow);


    %% =====================================================
    % DETERMINE CURRENT PHASE (folded into the title below, so
    % it can never visually collide with any other text object)
    % ======================================================

    if tNow < initialGuidanceTime

        phaseText = 'DIRECT TO TARGET';

    elseif tNow < transitionEnd

        phaseText = 'PPN TRANSITION';

    else

        phaseText = 'PPN PURSUIT';

    end


    %% =====================================================
    % MAIN 3D TITLE (two lines: phase, then telemetry)
    % ======================================================

title(ax3D, sprintf('%s | N = %g | Range = %.0f m | Time = %.2f s', phaseText, N, rangeHist(k), tNow), ...
    'FontSize',12,'Color','#3270C9', ...
'Units','normalized', 'Position',[0.2 0.90 0], ...
'HorizontalAlignment','center','VerticalAlignment','top');


    %% =====================================================
    % UPDATE FIGURE
    % ======================================================

    drawnow;


    %% =====================================================
    % ANIMATION SPEED
    % ======================================================

    pause(0.001);

end


%% =========================================================
% FINAL DISPLAY
% ==========================================================

set(hUAVTrail, ...
    'XData',uavPos(1,:), ...
    'YData',uavPos(2,:), ...
    'ZData',uavPos(3,:));


set(hTargetTrail, ...
    'XData',targetPos(1,:), ...
    'YData',targetPos(2,:), ...
    'ZData',targetPos(3,:));


%% =========================================================
% FINAL UAV ORIENTATION
% ==========================================================

finalVelocity = uavVel(:,end);

if norm(finalVelocity) > 1e-6

    finalDirection = ...
        finalVelocity / norm(finalVelocity);

else

    finalDirection = initialLOS;

end


Rfinal = directionRotation( ...
    [1;0;0], ...
    finalDirection);


Tfinal = eye(4);

Tfinal(1:3,1:3) = Rfinal;

Tfinal(1:3,4) = uavPos(:,end);


set(uavTransform, ...
    'Matrix',Tfinal);


%% =========================================================
% FINAL TITLE
% ==========================================================

title(ax3D, { ...
    sprintf('INTERCEPT | N = %g | R = %.2f m | t = %.3f s', ...
        N, rangeHist(end), time(end)) ...
    }, ...
    'FontSize',12,'Color','#3270C9','FontWeight','bold');
drawnow;


%% =========================================================
% FINAL RESULTS
% ==========================================================

fprintf('\n');
fprintf('=================================================\n');
fprintf('          INTERCEPTOR UAV SIMULATION\n');
fprintf('=================================================\n');
fprintf('Navigation Constant : N = %.1f\n',N);
fprintf('Initial UAV Speed   : %.3f m/s\n',initialSpeed);
fprintf('Intercept Time      : %.3f s\n',time(end));
fprintf('Final Range         : %.3f m\n',rangeHist(end));
fprintf('Closing Velocity    : %.3f m/s\n',VcHist(end));
fprintf('LOS Angular Rate    : %.6f rad/s\n',omegaHist(end));
fprintf('Final Acceleration  : %.3f m/s^2\n',accelHist(end));
fprintf('Final UAV Speed     : %.3f m/s\n', ...
    norm(uavVel(:,end)));
fprintf('=================================================\n');


%% =========================================================
% LOCAL FUNCTION
% CREATE INTERCEPTOR UAV (procedural fallback)
% ==========================================================

function createUAVModel(parent,scale)

    %% -----------------------------------------------------
    % UAV dimensions
    % ------------------------------------------------------

    L = scale;

    W = scale * 0.18;

    H = scale * 0.12;


    %% =====================================================
    % MAIN FUSELAGE
    % ======================================================

    x = [-L/2 L/2];

    y = [-W/2 W/2];

    z = [-H/2 H/2];


    vertices = [ ...
        x(1) y(1) z(1);
        x(2) y(1) z(1);
        x(2) y(2) z(1);
        x(1) y(2) z(1);
        x(1) y(1) z(2);
        x(2) y(1) z(2);
        x(2) y(2) z(2);
        x(1) y(2) z(2)];


    faces = [ ...
        1 2 3 4;
        5 6 7 8;
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8];


    patch( ...
        'Parent',parent, ...
        'Vertices',vertices, ...
        'Faces',faces, ...
        'FaceColor',[0.15 0.15 0.18], ...
        'EdgeColor','k');


    %% =====================================================
    % WINGS
    % ======================================================

    wingSpan = scale * 0.9;

    wingChord = scale * 0.35;


    verticesWing = [ ...
         0                 0             0;
        -wingChord         wingSpan/2    0;
        -wingChord*0.2     wingSpan/2    0;
         wingChord*0.25    0             0;

         0                 0             0;
        -wingChord        -wingSpan/2    0;
        -wingChord*0.2    -wingSpan/2    0;
         wingChord*0.25    0             0];


    facesWing = [ ...
        1 2 3 4;
        5 6 7 8];


    patch( ...
        'Parent',parent, ...
        'Vertices',verticesWing, ...
        'Faces',facesWing, ...
        'FaceColor',[0.25 0.25 0.28], ...
        'EdgeColor','k');


    %% =====================================================
    % VERTICAL TAIL
    % ======================================================

    tailX = -scale * 0.35;

    tailH = scale * 0.35;

    tailThickness = scale * 0.04;


    verticesTail = [ ...
        tailX-tailThickness  0  0;
        tailX+tailThickness  0  0;
        tailX+tailThickness  0  tailH;
        tailX-tailThickness  0  tailH];


    facesTail = [1 2 3 4];


    patch( ...
        'Parent',parent, ...
        'Vertices',verticesTail, ...
        'Faces',facesTail, ...
        'FaceColor',[0.20 0.20 0.22], ...
        'EdgeColor','k');


    %% =====================================================
    % NOSE
    % ======================================================

    noseLength = scale * 0.25;

    noseRadius = W * 0.5;

    n = 12;

    theta = linspace(0,2*pi,n+1);


    noseVertices = zeros(n+1,3);

    noseVertices(1,:) = ...
        [L/2+noseLength 0 0];


    for i = 1:n

        noseVertices(i+1,:) = ...
            [L/2 ...
             noseRadius*cos(theta(i)) ...
             noseRadius*sin(theta(i))];

    end


    noseFaces = zeros(n,3);


    for i = 1:n

        j = i+1;

        if j > n

            j = 1;

        end

        noseFaces(i,:) = ...
            [1 i+1 j+1];

    end


    patch( ...
        'Parent',parent, ...
        'Vertices',noseVertices, ...
        'Faces',noseFaces, ...
        'FaceColor',[0.65 0.65 0.68], ...
        'EdgeColor','none');

end


%% =========================================================
% LOCAL FUNCTION
% CREATE TARGET AIRCRAFT (procedural fallback, orange)
% ==========================================================

function createTargetAircraft(parent,scale)

    L = scale;


    %% =====================================================
    % FUSELAGE
    % ======================================================

    fuselageLength = L;

    fuselageWidth = L * 0.12;

    fuselageHeight = L * 0.12;


    x = [-fuselageLength/2 fuselageLength/2];

    y = [-fuselageWidth/2 fuselageWidth/2];

    z = [-fuselageHeight/2 fuselageHeight/2];


    vertices = [ ...
        x(1) y(1) z(1);
        x(2) y(1) z(1);
        x(2) y(2) z(1);
        x(1) y(2) z(1);
        x(1) y(1) z(2);
        x(2) y(1) z(2);
        x(2) y(2) z(2);
        x(1) y(2) z(2)];


    faces = [ ...
        1 2 3 4;
        5 6 7 8;
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8];


    patch( ...
        'Parent',parent, ...
        'Vertices',vertices, ...
        'Faces',faces, ...
        'FaceColor',[0.95 0.45 0.05], ...
        'EdgeColor','k');


    %% =====================================================
    % WINGS
    % ======================================================

    wingSpan = L * 0.75;

    wingBack = -L * 0.20;


    verticesWing = [ ...
         L*0.15   0             0;
         wingBack wingSpan/2    0;
        -L*0.35   wingSpan/2    0;
        -L*0.10   0             0;

         L*0.15   0             0;
         wingBack -wingSpan/2   0;
        -L*0.35  -wingSpan/2    0;
        -L*0.10   0             0];


    facesWing = [ ...
        1 2 3 4;
        5 6 7 8];


    patch( ...
        'Parent',parent, ...
        'Vertices',verticesWing, ...
        'Faces',facesWing, ...
        'FaceColor',[0.85 0.35 0.02], ...
        'EdgeColor','k');


    %% =====================================================
    % VERTICAL TAIL
    % ======================================================

    verticesTail = [ ...
        -L*0.40  0  0;
        -L*0.10  0  0;
        -L*0.30  0  L*0.30];


    facesTail = [1 2 3];


    patch( ...
        'Parent',parent, ...
        'Vertices',verticesTail, ...
        'Faces',facesTail, ...
        'FaceColor',[0.75 0.30 0.02], ...
        'EdgeColor','k');

end


%% =========================================================
% LOCAL FUNCTION
% ROTATION FROM ONE VECTOR TO ANOTHER
% ==========================================================

function R = directionRotation(v1,v2)

    v1 = v1 / norm(v1);

    v2 = v2 / norm(v2);


    %% -----------------------------------------------------
    % Rotation axis
    % ------------------------------------------------------

    v = cross(v1,v2);

    s = norm(v);

    c = dot(v1,v2);


    %% -----------------------------------------------------
    % Same direction
    % ------------------------------------------------------

    if s < 1e-10

        if c > 0

            R = eye(3);

        else

            % 180 degree rotation

            R = [ ...
                -1  0  0;
                 0  1  0;
                 0  0 -1];

        end

        return;

    end


    %% -----------------------------------------------------
    % Rodrigues rotation matrix
    % ------------------------------------------------------

    vx = [ ...
         0    -v(3)  v(2);
         v(3)  0    -v(1);
        -v(2)  v(1)  0];


    R = eye(3) + ...
        vx + ...
        vx^2 * ((1-c)/(s^2));

end


%% =========================================================
% LOCAL FUNCTION
% CREATE 3D BODY FROM AN STL FILE
% ==========================================================

function createSTLModel(parent, filename, desiredLength, faceColor, rotFix)
% Loads an STL, centers it, auto-scales it to desiredLength along its
% longest axis, applies a fixed orientation correction (rotFix), and
% attaches it as a patch under the given hgtransform. Designed to
% plug directly into this script's directionRotation()-based
% animation, which assumes the model's LOCAL +X axis is "forward".

    if nargin < 5 || isempty(rotFix)
        rotFix = eye(3);
    end

    [V, F] = readSTL(filename);

    % Center the mesh on its own centroid
    V = V - mean(V, 1);

    % Auto-scale so nose-to-tail length == desiredLength
    extents = max(V, [], 1) - min(V, [], 1);
    currentLength = max(extents);
    if currentLength < eps
        error('createSTLModel:degenerate', ...
            'STL bounding box has zero size: %s', filename);
    end
    V = V * (desiredLength / currentLength);

    % Apply orientation fix so nose = local +X
    V = (rotFix * V')';

    patch( ...
        'Parent', parent, ...
        'Vertices', V, ...
        'Faces', F, ...
        'FaceColor', faceColor, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.4, ...
        'SpecularStrength', 0.3);

end


%% =========================================================
% LOCAL FUNCTION
% READ AN STL FILE (BINARY OR ASCII) - NO TOOLBOX REQUIRED
% ==========================================================

function [vertices, faces] = readSTL(filename)
% [vertices, faces] = readSTL('model.stl')
%   vertices : Nx3 unique vertex list
%   faces    : Mx3 triangle index list (into vertices)
%
% Deliberately does NOT call MATLAB's built-in stlread (nor any File
% Exchange stlread), avoiding ambiguity when multiple versions of
% that function name exist on the user's path.

    if ~isfile(filename)
        error('readSTL:fileNotFound', 'Could not find file: %s', filename);
    end

    fid = fopen(filename, 'r');
    if fid == -1
        error('readSTL:fileNotFound', 'Could not open file: %s', filename);
    end

    % Peek at first line to guess ascii vs binary. Ascii STLs start
    % with the literal text "solid". Some binary files also happen to
    % start with "solid" in their 80-byte header, so this is
    % confirmed below by checking for readable "facet" text.
    firstLine = fgetl(fid);
    isAsciiGuess = ischar(firstLine) && startsWith(strtrim(firstLine), 'solid');
    fclose(fid);

    isBinary = ~isAsciiGuess;

    if isAsciiGuess
        fid = fopen(filename, 'r');
        chunk = fread(fid, 512, '*char')';
        fclose(fid);
        if ~contains(chunk, 'facet')
            isBinary = true;
        end
    end

    if isBinary
        [vertices, faces] = localReadBinary(filename);
    else
        [vertices, faces] = localReadAscii(filename);
    end

end


function [vertices, faces] = localReadBinary(filename)

    fid = fopen(filename, 'r');
    fread(fid, 80, 'uint8');              % 80-byte header (ignored)
    numTri = fread(fid, 1, 'uint32');

    rawVerts = zeros(numTri * 3, 3);

    for i = 1:numTri
        fread(fid, 3, 'float32');         % normal vector (ignored)
        v1 = fread(fid, 3, 'float32')';
        v2 = fread(fid, 3, 'float32')';
        v3 = fread(fid, 3, 'float32')';
        fread(fid, 1, 'uint16');          % attribute byte count

        idx = (i-1)*3;
        rawVerts(idx+1,:) = v1;
        rawVerts(idx+2,:) = v2;
        rawVerts(idx+3,:) = v3;
    end
    fclose(fid);

    faces = reshape(1:size(rawVerts,1), 3, numTri)';
    [vertices, faces] = localWeldVertices(rawVerts, faces);

end


function [vertices, faces] = localReadAscii(filename)

    fid = fopen(filename, 'r');
    txt = fread(fid, '*char')';
    fclose(fid);

    tok = regexp(txt, 'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', 'tokens');

    numVerts = numel(tok);
    if mod(numVerts, 3) ~= 0
        error('readSTL:badAscii', ...
            'Malformed ASCII STL: vertex count not divisible by 3.');
    end

    rawVerts = zeros(numVerts, 3);
    for i = 1:numVerts
        rawVerts(i,:) = cellfun(@str2double, tok{i});
    end

    numTri = numVerts / 3;
    faces = reshape(1:numVerts, 3, numTri)';
    [vertices, faces] = localWeldVertices(rawVerts, faces);

end


function [vertices, faces] = localWeldVertices(rawVerts, faces)
    % Merge duplicate vertices (shared triangle edges)
    [vertices, ~, ic] = unique(round(rawVerts, 6), 'rows', 'stable');
    faces = reshape(ic(faces(:)), size(faces));
end
