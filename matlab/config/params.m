function P = params()
%PARAMS  All numbers for the eye-tracking rig (simulation and hardware), from our order list.
%   Values marked [PLACEHOLDER] are NOT in any datasheet -> measure them.
%   Values marked [CHECK] are interpretations of a datasheet -> verify.
%
%   Axis order everywhere in this project:
%     1 = Camera X  : LINEAR carriage on 2020 V-slot, V-wheels, GT2 belt,
%                     FIT0186 geared DC motor + encoder, MDD10A driver
%     2 = Illum A Y : tilt, SER0006 micro servo
%     3 = Illum B Y : tilt, SER0006 micro servo
%   Controller board: NUCLEO-H753ZI (STM32H753, Cortex-M7)

%% Timing / architecture
%  'matlab' : PID runs in MATLAB on the PC, Nucleo is only an I/O bridge over
%             USB serial (team choice). 100 Hz loop, USB adds delay.
%  'nucleo' : PID runs on the Nucleo at 1 kHz (for comparison).
P.loopMode   = 'matlab';
if strcmp(P.loopMode, 'matlab')
    P.Ts = 10e-3;  bwCam = 2;     % 2 Hz keeps PM ~40 deg with 15 ms total delay
else
    P.Ts = 1e-3;   bwCam = 6;
end
P.nSub       = round(P.Ts/1e-4);  % plant sub-step 0.1 ms
P.calcDelay  = true;   % voltage computed at k is applied at k+1
P.g          = 9.81;

%% Power + driver
%  PSU   : Mean Well LRS-150-12 -> 12 V, 12.5 A
%  Driver: Cytron MDD10A, 2 ch, 10 A/ch, 5-30 V, sign-magnitude PWM+DIR,
%          3.3 V logic OK (MD10C also ordered: single channel, spare)
P.psu.V    = 12;
P.psu.Imax = 12.5;
P.drv.fPWM = 20e3;            % [Hz]
P.drv.pwmCounts = 12000;      % STM32 timer: 240 MHz / 20 kHz      [CHECK clock config]
P.drv.Imax = 10;              % MDD10A continuous per channel [A]

%% FIT0186 geared DC motor (DFRobot wiki):
%  12 V, 43.8:1, no-load 251 rpm @ 350 mA, stall 18 kg*cm @ 7 A,
%  encoder 16 CPR motor shaft = 700 CPR at gearbox shaft, 6 mm D-shaft.
%  Derived: R = 12/7, Ke from no-load speed, gearbox efficiency from
%  stall torque vs. Ke (Kt_out/Ke_out).
N      = 43.8;
wNL    = 251*2*pi/60;               % output no-load speed [rad/s]
R      = 12/7;                      % [Ohm]
KeOut  = (12 - 0.35*R)/wNL;         % output-side back-EMF const [V s/rad]
KtOut  = 18*9.81/100/7;             % output-side torque const from stall [Nm/A]
mot.N    = N;
mot.R    = R;
mot.Ke   = KeOut/N;                 % motor-side
mot.Kt   = KeOut/N;                 % motor-side (ideal Kt = Ke in SI)
mot.eta  = KtOut/KeOut;             % gearbox efficiency (~0.58)
mot.T0   = KtOut*0.35;              % no-load friction torque at output [Nm]
mot.L    = 1.0e-3;                  % [H]          [PLACEHOLDER - measure with LCR meter]
mot.Jm   = 1.5e-6;                  % rotor inertia [kg m^2] [PLACEHOLDER - identify, see sysid_step.m]
mot.bm   = 0;                       % viscous part folded into T0
mot.Vmax = P.psu.V;
mot.cpr  = 700/N;                   % counts per MOTOR rev so that output = 700 CPR [CHECK x1/x4]
P.mot = mot;

%% Belt drive of the camera carriage
%  GT2 belt (2 mm pitch), 40-tooth pulley on the motor, smooth idler at the
%  other end -> carriage moves 80 mm per output-shaft revolution.
%  NOTE: pulley bore 6.35 mm on a 6 mm D-shaft -> use a sleeve / shim.
P.belt.r    = 40*2e-3/(2*pi);      % effective pulley radius [m] (12.7 mm)
P.belt.mCar = 0.30;                % carriage + camera + illuminators [kg]  [PLACEHOLDER - weigh it]
P.belt.Fc   = 2.0;                 % V-wheel rolling friction [N]          [PLACEHOLDER]
P.belt.xMin = -0.15; P.belt.xMax = 0.15;   % usable travel (software limits) [m]  [PLACEHOLDER - rail length]
P.belt.xStop = P.belt.xMin - 0.005;          % carriage position at the MIN mechanical stop [m] [MEASURE]

%% Homing. The team has ONE endstop -> mount it at the MIN end and home on it.
%  Without it: P.home.method = 'stall' (drive slowly into the mechanical stop).
P.home.method = 'endstop';    % 'endstop' | 'stall'
P.home.duty   = 0.2;          % fraction of 12 V while homing (2.4 V, ~1.4 A at stall)
P.home.tStall = 0.2;          % encoder still for this long = at the stop [s]
P.endstops   = [true false];  % [MIN MAX] wired? must match USE_END_MIN / USE_END_MAX in the firmware

%% SER0006 micro servo (DFRobot / Farnell): 4.8-6 V, 0.156 Nm @6 V
%  (1.3 kg*cm used = conservative @4.8 V), 0.12 s/60 deg @4.8 V, dead band 7 us.
%  !! needs its own 5-6 V supply (not 12 V, not MDD10A)
srv.tauStall = 1.3*9.81/100;
srv.wmax     = (60*pi/180)/0.12;
srv.deadUs   = 7;
srv.pulseMin = 500;  srv.pulseMax = 2500;   % [us] for 0..180 deg  [CHECK by measuring]
srv.range    = pi;
srv.framePer = 20e-3;              % 50 Hz
srv.Kin      = 40;                 % internal loop gain [1/s]     [PLACEHOLDER]
srv.tauW     = 0.015;              % speed lag [s]                [PLACEHOLDER]
srv.usRes    = 1;                  % STM32 timer gives <=1 us easily
srv.zeroUs   = [1500 1500];        % pulse [us] at which each LED points horizontal [CALIBRATE]
srv.dir      = [1 1];              % +1/-1: does a longer pulse tilt up?           [CALIBRATE]
P.srv = srv;

%% Axes
%  type 'dc' (our PID on the Nucleo) or 'servo'
%  r    : pulley radius for linear axes [m], 0 for rotary
%  JL   : load inertia at output shaft [kg m^2],  Tc: Coulomb friction at output [Nm]
%  m, l : mass and CoM offset from tilt axis (gravity = m g l cos(theta))
%  bwHz, zeta, alpha : pole-placement spec (dc only)
%  vmax : reference slew limit [rad/s at the output shaft]
r = P.belt.r;
ax = struct('name',{}, 'type',{}, 'r',{}, 'JL',{}, 'bL',{}, 'Tc',{}, 'm',{}, 'l',{}, ...
            'bwHz',{}, 'zeta',{}, 'alpha',{}, 'vmax',{});
ax(1) = struct('name','Camera X (linear)','type','dc','r',r, ...
               'JL',P.belt.mCar*r^2,'bL',0,'Tc',mot.T0 + P.belt.Fc*r, ...
               'm',0,'l',0,'bwHz',bwCam,'zeta',0.8,'alpha',1.0,'vmax',0.8*wNL);
ax(2) = struct('name','Illum A Y (tilt)','type','servo','r',0,'JL',2e-6,'bL',0,'Tc',0, ...
               'm',0.01,'l',0.01,'bwHz',NaN,'zeta',NaN,'alpha',NaN,'vmax',3);
ax(3) = struct('name','Illum B Y (tilt)','type','servo','r',0,'JL',2e-6,'bL',0,'Tc',0, ...
               'm',0.01,'l',0.01,'bwHz',NaN,'zeta',NaN,'alpha',NaN,'vmax',3);
%  servo load: 5 mm IR LED + small perfboard piece, ~10 g at 1 cm   [PLACEHOLDER]
P.ax = ax;
P.useGravFF = true;
P.bSP       = 0.6;     % set-point weight on P-term

%% IR LED: Wurth 15400585A3590, 850 nm, 85 mW/sr, If 50 mA, Vf max 2.2 V
%  "35 deg" angle of half intensity: treated as FULL angle -> +-17.5 deg  [CHECK datasheet: 2*theta_1/2?]
P.led.halfAngleDeg = 17.5;
P.led.If  = 0.050; P.led.VfTyp = 1.5; P.led.VfMax = 2.2;  % VfTyp [CHECK]
P.led.Rser = 68;  P.led.Prated = 0.25;                     % Yageo MFR-25 68R
P.led.n    = 4;    % schematic: 4 IR LEDs, each 68R + NPN switch (470R base, 10k pull-down)
P.led.on   = [1 1 1 1];  % which LEDs to switch on while tracking
%  Position of each LED relative to the camera, on the carriage [m]   [CHECK WITH THE BUILD]
%  LED 1, 2: on the servos (tilt toward the eyes). LED 3, 4: fixed next to the camera.
P.led.pos   = [-0.06  0.06 -0.025 0.025;
               -0.03 -0.03  0.000 0.000;
                0     0     0     0    ];
P.led.servo = [1 2 0 0];   % LED i is tilted by servo k (0 = fixed, pointing +z)

%% Cross-axis coupling: carriage acceleration -> disturbance on the tilt servos
c = 0.05;                                          % [PLACEHOLDER]
P.kc = [0 0 0; c 0 0; c 0 0];

%% Geometry, world frame [m]: x right, y up, z toward the user.
%  Camera on the carriage at (xCar, 0, 0) looking +z (no rotation).
%  Illuminators also on the carriage, offset from the camera       [CHECK WITH TEAM]
%  (camera + both lights must NOT be collinear, see gaze_sim)
P.geo.cam        = [0; 0; 0];
P.geo.illumA_off = P.led.pos(:,1);               % servo LED A
P.geo.illumB_off = P.led.pos(:,2);               % servo LED B
P.geo.illumA = P.geo.cam + P.geo.illumA_off;      % at carriage x = 0 (gaze_sim)
P.geo.illumB = P.geo.cam + P.geo.illumB_off;
P.geo.cam2   = [0.10; 0; 0];                      % hypothetical 2nd camera, stereo comparison only

%% Serial link to the Nucleo (ST-LINK virtual COM port = USART3 on Nucleo-144)
P.link.port = "COM5";      % Windows: check Device Manager; macOS/Linux: "/dev/tty.usbmodem..." [CHECK]
P.link.baud = 921600;      % [CHECK] must match the firmware
P.link.watchdog = 0.1;     % firmware stops the motor if no command for 100 ms
P.link.timeout  = 0.05;    % serial read timeout [s]

%% Camera: Waveshare OV9281 1MP Mono USB Camera (A)
%  mono, NO IR filter (sees 850 nm), global shutter, USB2 MJPG, 1280x720 @120 fps,
%  1/4" sensor, EFL 1.65 mm, FOV 105 deg, distortion <10 %, fixed focus.
P.cam.pixel = 3e-6;                 % OV9281 pixel pitch [m]           [CHECK]
P.cam.f_px  = 1.65e-3/P.cam.pixel;  % = 550 px
P.cam.W     = 1280;
P.cam.H     = 720;
P.cam.fps   = 120;
P.cam.zEye  = 0.60;                 % fallback eye distance if both eyes are not seen [m]
P.cam.latency = 0.020;              % exposure + MJPG + USB + decode [s]    [PLACEHOLDER - measure]
P.cam.sigmaPx = 0.5;                % pupil-centre noise [px]               [PLACEHOLDER]
P.cam.zRelErr = 0.05;               % distance estimate error (from eye spacing) [-]
P.cam.blinks  = [1.5 4.6];          % blink start times in the scenario [s] (150 ms each)
%  Frame source for camera_open: 'webcam' (real OV9281), 'video' (recorded file),
%  'images' (folder of PNG/JPG), 'synthetic' (rendered eyes, no hardware)
P.cam.source     = 'webcam';
P.cam.deviceName = '';              % part of the webcam name; '' = first camera found [CHECK webcamlist]
P.cam.videoFile  = '';              % for 'video'
P.cam.imageDir   = '';              % for 'images'
P.cam.every      = 3;               % in run_rig: grab a frame every N control periods (~33 Hz)
P.cam.flipX      = false;           % mirror the image if the camera is mounted upside down
P.cam.flipY      = false;

%% Pupil + glint detector (eye_detection/eye_detect.m)   [PLACEHOLDER - tune with tune_detector.m]
%  With f = 550 px a 4 mm pupil is only ~3.7 px across at 60 cm, so everything
%  is tuned for SMALL dark blobs. Glints of all LEDs merge into ~1 bright spot.
P.det.down        = 2;       % full-frame search on a 2x min-downsampled image
P.det.bgWin       = 15;      % local background window [px] (full resolution)
P.det.darkK       = 0.55;    % pupil pixel: I < bg - darkK*(bg - Imin_local) (relative darkness)
P.det.absDark     = 90;      % and I < absDark (0..255)
P.det.minBg       = 70;      % and the local background is lit (pupil sits in a lit face)
P.det.darkBlobK   = 0.70;    % blob mean must be < (1 - darkBlobK) * background (pupils are much darker than brows)
P.det.rPupilMm    = [1.0 4.0];  % pupil radius range [mm]
P.det.zRange      = [0.30 0.70];% supported eye distance [m]: beyond ~0.7 m the pupil is < 3 px (see docs)
P.det.minCirc     = 0.35;    % min circularity (minor/major axis ratio)
P.det.minEncl     = 0.80;    % fraction of a ring around the blob that must be brighter
P.det.minConf     = 0.30;    % blobs below this confidence are ignored
P.det.ipd         = 0.063;   % interpupillary distance [m] (adult mean; measure the user!)
P.det.ipdTol      = 0.30;    % accepted relative deviation of the pair spacing
P.det.maxRollDeg  = 30;      % max head roll for an eye pair
P.det.roiHalf     = [90 45]; % search window half-size around each tracked eye [px] (x, y)
P.det.glintThr    = 0.85;    % glint pixel: I > glintThr*max in the eye window, and
P.det.glintMin    = 180;     % I > glintMin (0..255)
P.det.glintRadK   = 2.2;     % glints searched within max(3 px, glintRadK * pupil radius)
P.det.maxGlints   = 4;
P.det.maxGlintArea= 12;      % [px] larger bright areas are sclera/skin, not glints
P.det.whichEye    = 'right'; % eye used when only one is wanted (image right)

%% Eye tracker (eye_detection/eye_tracker.m)
P.trk.blinkMax    = 0.40;    % both eyes missing shorter than this = blink [s]
P.trk.lostAfter   = 1.0;     % missing longer -> 'lost', full-frame search [s]
P.trk.maxJumpM    = 0.02;    % a detection further than this from the prediction is rejected [m]
P.trk.vHeadMax    = 0.5;     % max head speed, widens the gate while an eye is unseen [m/s]

%% Head estimate (eye_detection/head_estimate.m)
P.head.zDefault   = 0.60;    % distance used until both eyes have been seen [m]
P.head.zAlpha     = 0.2;     % low-pass on the distance from the eye spacing

%% Eye-tracking software options (to recover performance with the wide lens)
P.trk.useFilter = true;     % Kalman filter on the eye position (+ velocity)
P.trk.useVelFF  = true;     % carriage velocity feed-forward from the filter
P.trk.qAcc      = 0.5;      % filter process noise: head acceleration PSD [(m/s^2)^2/Hz] (tuned)
P.trk.bSP       = 1.0;      % no set-point weighting while tracking (b<1 adds lag on ramps)
P.trk.jumpGate  = 4;        % innovation > jumpGate*sigma + jumpMin = jump, no velocity update
P.trk.jumpMin   = 0.001;    % [m]
P.trk.vffTau    = 0.15;     % low-pass on the velocity feed-forward [s] (saccades, noise)

%% Head / eye motion scenario
P.traj.Tend  = 6;
P.traj.p0    = [0; 0.05; 0.60];
P.traj.Ax    = 0.08; P.traj.fx = 0.3;
P.traj.Ay    = 0.03; P.traj.fy = 0.2;
P.traj.Az    = 0.05; P.traj.fz = 0.15;
P.traj.tJump = 3.0;  P.traj.jump = [0.05; -0.02; 0];

%% Step test: axis 1 in metres of carriage travel, axes 2-3 in degrees
P.step.size   = [0.05, 10*pi/180, 10*pi/180];   % converted to output angle in main
P.step.tStart = [0.1 0.9 1.4];
P.step.Tend   = 1.9;

%% Screen for gaze calibration (gaze_estimation)       [MEASURE the monitor + mounting]
%  Screen plane z = 0 (same as the camera), centre at (xc, yc). Units: metres.
P.screen.size    = [0.53 0.30];     % visible width, height [m] (24" 16:9)
P.screen.centre  = [0; 0.20; 0];    % screen centre relative to the camera at x = 0 [m]
P.screen.res     = [1920 1080];     % pixels
P.screen.monitor = 1;               % which monitor to use for the calibration window

%% Gaze calibration (PCCR + 2nd-order polynomial)
P.gcal.grid       = [3 3];          % calibration points (columns x rows)
P.gcal.margin     = 0.10;           % distance of outer points from the screen edge (fraction)
P.gcal.settle     = 0.8;            % wait after a point appears before recording [s]
P.gcal.record     = 1.0;            % record time per point [s]
P.gcal.order      = 2;              % polynomial order (1 or 2)
P.gcal.minSamples = 10;             % valid frames needed per point
P.gcal.file       = 'gaze_calibration.mat';   % stored in matlab/config/local (git-ignored)
P.gcal.smooth     = 0.3;            % low-pass on the gaze point while tracking (0..1)

%% Live view (visualization)
P.viz.rate   = 10;     % figure updates per second during run_rig / run_eye_demo
P.viz.show   = true;

%% Synthetic scene (camera source 'synthetic', tests)
P.syn.noise   = 6;      % image noise std (0..255)
P.syn.rollAmp = 8;      % head roll amplitude [deg]
P.syn.gazeDwell = 1.0;  % time the virtual user looks at each target [s]
P.syn.distractors = true;   % eyebrows + nostrils as false dark blobs
P.syn.glints  = true;   % draw corneal reflections

%% Eye model + gaze Monte-Carlo
P.eye.R      = 7.8e-3;
P.eye.K      = 4.2e-3;
P.gaze.sigmaPx = [0 0.1 0.25 0.5 1 2];
P.gaze.nTrials = 300;
P.gaze.seed    = 1;

%% Local overrides (COM port, measured values, ...) -> matlab/config/local/params_local.m
%  (git-ignored; copy params_local_example.m there)
if exist('params_local', 'file') == 2 && isempty(getenv('GCCE_NO_LOCAL'))   % tests set GCCE_NO_LOCAL
    P = params_local(P);
end
end
