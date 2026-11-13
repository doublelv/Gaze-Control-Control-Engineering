function P = params()
%PARAMS  All numbers for the eye-tracking rig (simulation and hardware), from our order list.
%   Values marked [PLACEHOLDER] are NOT in any datasheet -> measure them.
%   Values marked [CHECK] are interpretations of a datasheet -> verify.
%
%   Axis order everywhere in this project:
%     1 = Camera X  : LINEAR carriage on 2020 V-slot, V-wheels, GT2 belt,
%                     FIT0186 geared DC motor + encoder, MD10C driver
%     2 = Illum A   : pan, servo 1 (PC6) on the MAX-side (user's LEFT) leg of the frame
%     3 = Illum B   : pan, servo 2 (PC7) on the MIN-side (user's RIGHT) leg of the frame
%     (the servos are fixed to the U-frame, NOT on the carriage; see P.led)
%   Controller board: NUCLEO-H753ZI (STM32H753, Cortex-M7)

%% Timing / architecture
%  'matlab' : PID runs in MATLAB on the PC, Nucleo is only an I/O bridge over
%             USB serial (team choice). 50 Hz loop, USB adds delay.
%  'nucleo' : PID runs on the Nucleo at 1 kHz (for comparison).
P.loopMode   = 'matlab';
if strcmp(P.loopMode, 'matlab')
    % Measured on the rig: loop median ~46 ms (camera + face detection), so
    % 20 ms is the target and 1 Hz the bandwidth that stays calm at that rate.
    % (At 1 Hz pole placement gives Kd < 0; design_pid then sets Kd from a
    %  phase-margin rule, see P.ax(1).pmTarget.)
    P.Ts = 20e-3;  bwCam = 1;
else
    P.Ts = 1e-3;   bwCam = 6;
end
P.nSub       = round(P.Ts/1e-4);  % plant sub-step 0.1 ms
P.calcDelay  = true;   % voltage computed at k is applied at k+1
P.g          = 9.81;

%% Power + driver
%  PSU   : Mean Well LRS-150-12 -> 12 V, 12.5 A
%  Driver: Cytron MD10C (as built), 1 ch, 13 A continuous, 5-30 V, sign-magnitude
%          PWM+DIR, 3.3 V logic OK   [CHECK 13 A / 5-30 V with the team]
P.psu.V    = 12;
P.psu.Imax = 12.5;
P.drv.fPWM = 20e3;            % [Hz]
P.drv.pwmCounts = 12000;      % STM32 timer: 240 MHz / 20 kHz      [CHECK clock config]
P.drv.Imax = 10;              % [A] conservative limit for the stall-current check (MD10C datasheet: 13 A [CHECK])

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
mot.Jm   = 1.08e-6;                 % rotor inertia [kg m^2] (identified on the rig, run_rig TASK = 'sysid')
mot.bm   = 0;                       % viscous part folded into T0
mot.Vmax = P.psu.V;
mot.cpr  = 2800/N;                   % counts per MOTOR rev so that output = 700 CPR [CHECK x1/x4]
P.mot = mot;

%% Belt drive of the camera carriage
%  GT2 belt (2 mm pitch), 40-tooth pulley on the motor, smooth idler at the
%  other end -> carriage moves 80 mm per output-shaft revolution.
%  NOTE: pulley bore 6.35 mm on a 6 mm D-shaft -> use a sleeve / shim.
P.belt.r    = 40*2e-3/(2*pi);      % effective pulley radius [m] (12.7 mm)
P.belt.mCar = 0.30;                % carriage + camera + illuminators [kg]  [PLACEHOLDER - weigh it]
P.belt.Fc   = 2.0;                 % V-wheel rolling friction [N]          [PLACEHOLDER]
%  Measured: the camera moves 0.65 m from the MIN stop to the MAX stop.
%  x = 0 is the middle of the travel. Software limits P.belt.margin from each stop;
%  beyond a limit run_rig never drives further outward (encoder guard).
P.belt.travel = 0.65;                        % stop to stop [m] (measured)
P.belt.margin = 0.05;                        % software limit to mechanical stop [m]
P.belt.xStop = -P.belt.travel/2;             % carriage position at the MIN mechanical stop [m]
P.belt.xMin = P.belt.xStop + P.belt.margin; P.belt.xMax = -P.belt.xStop - P.belt.margin;   % software limits [m]

%% Homing. The team has ONE endstop -> mount it at the MIN end and home on it.
%  Without it: P.home.method = 'stall' (drive slowly into the mechanical stop).
P.home.method = 'stall';    % 'endstop' | 'stall'
P.home.duty   = 0.2;          % fraction of 12 V while homing (2.4 V, ~1.4 A at stall)
P.home.tStall = 0.1;          % encoder still for this long = at the stop [s] (short: light touch)
P.home.tBack  = 0.15;         % after each stop: drive back this long so it does not stay pressed [s]
P.home.bothEnds  = true;      % stall homing: MIN stop, then MAX stop -> measured travel, limits centred on it
P.home.travelTol = 0.05;      % measured travel must be within this of P.belt.travel, else STOP [m]
P.home.tCheck = 0.3;          % pre-check: push toward MAX this long before homing [s]
P.home.minCheckCounts = 100;  % ... and the encoder must count up at least this much (~3 mm)
P.endstops   = [false false];  % [MIN MAX] wired? must match USE_END_MIN / USE_END_MAX in the firmware

%% SER0006 micro servo (DFRobot / Farnell): 4.8-6 V, 0.156 Nm @6 V
%  (1.3 kg*cm used = conservative @4.8 V), 0.12 s/60 deg @4.8 V, dead band 7 us.
%  !! needs its own 5-6 V supply (not 12 V, not the motor driver)
srv.tauStall = 1.3*9.81/100;
srv.wmax     = (60*pi/180)/0.12;
srv.deadUs   = 7;
srv.pulseMin = 500;  srv.pulseMax = 2500;   % [us] for 0..180 deg  [CHECK by measuring]
srv.range    = pi;
srv.framePer = 20e-3;              % 50 Hz
srv.Kin      = 40;                 % internal loop gain [1/s]     [PLACEHOLDER]
srv.tauW     = 0.015;              % speed lag [s]                [PLACEHOLDER]
srv.usRes    = 1;                  % STM32 timer gives <=1 us easily
srv.zeroUs   = [1600 1930];        % pulse [us] at which each LED points straight ahead (+z, at the user)
                                   %   measured with servo_test 2026-11-09 (servo 1 = PC6, servo 2 = PC7)
srv.dir      = [1 -1];             % +1 if a longer pulse turns the LED toward +x (MAX side), else -1
                                   %   measured with servo_test 2026-11-09
P.srv = srv;

%% Axes
%  type 'dc' (our PID on the Nucleo) or 'servo'
%  r    : pulley radius for linear axes [m], 0 for rotary
%  JL   : load inertia at output shaft [kg m^2],  Tc: Coulomb friction at output [Nm]
%  m, l : mass and CoM offset from tilt axis (gravity = m g l cos(theta)); 0 for pan axes
%  bwHz, zeta, alpha : pole-placement spec (dc only)
%  vmax : reference slew limit [rad/s at the output shaft]
r = P.belt.r;
ax = struct('name',{}, 'type',{}, 'r',{}, 'JL',{}, 'bL',{}, 'Tc',{}, 'm',{}, 'l',{}, ...
            'bwHz',{}, 'zeta',{}, 'alpha',{}, 'vmax',{}, 'pmTarget',{}, 'TsPM',{});
ax(1) = struct('name','Camera X (linear)','type','dc','r',r, ...
               'JL',P.belt.mCar*r^2,'bL',0,'Tc',mot.T0 + P.belt.Fc*r, ...
               'm',0,'l',0,'bwHz',bwCam,'zeta',1.5,'alpha',1.0,'vmax',0.20/r, ...
               'pmTarget',50,'TsPM',0.046);
%  zeta 1.5: at 1 Hz the design gives Kd < 0 -> PI only; the higher zeta raises Kp
%  (1.48 -> 2.28 V/rad). With set-point weight b = 0.6 (P.trk.bSP) a 50 mm step then
%  has < 3 % overshoot and ~1 s settling (5 %) for loops of 20-56 ms in the model
%  (make_paper_figures, compare_log_sim). On the rig (run_rig TASK = 'step',
%  2026-11-09, 20 ms loop, camera off): 0 % overshoot in 9 of 9 steps.
%  The first rig runs used zeta 0.8 / b = 1 (model: ~40 % overshoot).
%  vmax: carriage at most 0.20 m/s (value set on the rig)
%  pmTarget/TsPM: D gain from the phase-margin rule in design_pid: Kd is raised until
%  the phase margin at the measured median loop period (46 ms, camera on) is 50 deg
%  (PI only: 42 deg). The D term (on the measurement, filtered) adds phase lead that
%  makes up for part of the loop delay -> full PID. Model: 2.6 % overshoot, 0.86 s (5 %).
ax(2) = struct('name','Illum A (pan)','type','servo','r',0,'JL',2e-6,'bL',0,'Tc',0, ...
               'm',0,'l',0,'bwHz',NaN,'zeta',NaN,'alpha',NaN,'vmax',3,'pmTarget',NaN,'TsPM',NaN);
ax(3) = struct('name','Illum B (pan)','type','servo','r',0,'JL',2e-6,'bL',0,'Tc',0, ...
               'm',0,'l',0,'bwHz',NaN,'zeta',NaN,'alpha',NaN,'vmax',3,'pmTarget',NaN,'TsPM',NaN);
%  servo load: 5 mm IR LED + small holder, ~10 g; vertical servo axis (pan) -> no gravity torque
P.ax = ax;
P.useGravFF = true;
P.bSP       = 0.6;     % set-point weight on P-term

%% IR LED: Wurth 15400585A3590, 850 nm, 85 mW/sr, If 50 mA, Vf max 2.2 V
%  "35 deg" angle of half intensity: treated as FULL angle -> +-17.5 deg  [CHECK datasheet: 2*theta_1/2?]
P.led.halfAngleDeg = 17.5;
P.led.If  = 0.050; P.led.VfTyp = 1.5; P.led.VfMax = 2.2;  % VfTyp [CHECK]
P.led.Rser = 68;  P.led.Prated = 0.25;                     % Yageo MFR-25 68R
P.led.n    = 4;    % LED channels 1-4 in the schematic (each 68R + NPN switch, 470R base, 10k pull-down)
%   LED i = J10 'IR LED i' = Nucleo LED_i (PF3, PD15, PD14, PA4) = schematic D(5-i)
%   (J10 'IR LED 1' drives D4 ... 'IR LED 4' drives D1). See docs/hardware/hardware.md.
%   As built there are 6 LEDs: LED_5 (PB15) and LED_6 (PA6) were added later as the
%   second LED on servo 1 / 2 (P.led.twin) and switch together with LED 1 / 2.
P.led.on   = [1 1 1 1];  % which LEDs to switch on (calibration, demo and tracking)
%  NOTE: the gaze feature uses the centroid of ALL glints. The frame LEDs (1, 2)
%  do not move with the camera, so their glints shift when the head moves
%  sideways -> gaze bias (test_gaze uses [0 0 1 1]). If the gaze point is off,
%  try P.led.on = [0 0 1 1] in params_local.m and recalibrate.
%
%  LED layout (as built): LED 1, 2 on the pan servos on the two legs of the
%  U-frame (fixed to the frame); LED 3, 4 fixed next to the camera on the carriage.
P.led.servo   = [1 2 0 0];               % LED i is panned by servo k (0 = no servo, points +z)
P.led.onFrame = [true true false false]; % true: fixed to the frame, false: rides on the carriage
P.led.twin    = [5 6 0 0];               % second LED on servo 1 / 2: Nucleo LED_5 (PB15), LED_6 (PA6), firmware with LED_5/LED_6
P.led.aim     = 'pan';                   % servo axis: 'pan' (vertical axis, as built) | 'tilt'
%  P.led.pos [m], one column per LED, rows x/y/z:
%   carriage LED: offset from the camera lens (x along the rail, y up, z toward the user)
%   frame LED   : x = distance along the rail from the camera lens WHEN THE CARRIAGE
%                 IS AT THE MIN STOP (where stall homing ends) to the servo shaft,
%                 positive toward the MAX end (negative if the leg is beyond the stop);
%                 y = height of the LED above the camera lens (negative = below);
%                 z = how far the LED is in front of the lens (toward the user).
%   -> world position: see led_world_pos.m
%  Measured: servo LEDs 26 cm above the table, lens 36.5 cm; each servo LED is
%  8 cm outside the lens when the carriage is at that end; 0.5 cm in front.
%  Camera LEDs 1.5 cm left/right of the lens, 1 cm in front.
%  Servo 1 (PC6, LED 1) is on the user's LEFT leg = MAX end (motor side), servo 2
%  (PC7, LED 2) on the RIGHT leg = MIN end (servo_test 2026-11-09).
P.led.pos = [P.belt.travel+0.080  -0.080  -0.015  0.015;
             -0.105  -0.105                 0.000  0.000;
              0.005   0.005                 0.010  0.010];

%% Cross-axis coupling: carriage acceleration -> disturbance on the servos.
%  The servos are on the frame, not on the carriage -> no coupling.
P.kc = zeros(3);

%% Geometry, world frame [m]: x along the rail (+ = +encoder = MAX side), y up, z toward the user.
%  Camera on the carriage at (xCar, 0, 0) looking +z (no rotation).
%  LED positions in this frame: led_world_pos(P, xCar)
%  (camera + both lights must NOT be collinear, see gaze_sim)
P.geo.cam    = [0; 0; 0];
P.geo.cam2   = [0.10; 0; 0];                      % hypothetical 2nd camera, stereo comparison only

%% Serial link to the Nucleo (ST-LINK virtual COM port = USART3 on Nucleo-144)
P.link.port = "COM5";      % Windows: check Device Manager; macOS/Linux: "/dev/tty.usbmodem..." [CHECK]
P.link.baud = 921600;      % [CHECK] must match the firmware
P.link.watchdog = 0.1;     % firmware stops the motor if no command for 100 ms
P.link.keepAlive = 0.04;   % run_rig: re-send the last command after this long inside a camera loop [s]
P.link.timeout  = 0.05;    % serial read timeout [s]
P.link.maxCountRate = 1.5*wNL/(2*pi)*2800;   % encoder plausibility: counts/s above this = jump (Nucleo reset?) -> STOP

%% Camera: Waveshare OV9281 1MP Mono USB Camera (A)
%  mono, NO IR filter (sees 850 nm), global shutter, USB2 MJPG, 1280x720 @120 fps,
%  1/4" sensor, EFL 1.65 mm, FOV 105 deg, distortion <10 %, fixed focus.
P.cam.pixel = 3e-6;                 % OV9281 pixel pitch [m]           [CHECK]
P.cam.f_px  = 1.65e-3/P.cam.pixel;  % = 550 px
P.cam.W     = 1280;
P.cam.H     = 720;
P.cam.fps   = 120;
P.cam.zEye  = 0.60;                 % fallback eye distance if both eyes are not seen [m]
P.cam.latency = 0.10;               % exposure + MJPG + USB + decode [s] (value set on the rig, not measured)
%  (run_rig serial: the carriage position at t - latency is used for each frame)
P.cam.sigmaPx = 0.5;                % pupil-centre noise [px]               [PLACEHOLDER]
P.cam.zRelErr = 0.05;               % distance estimate error (from eye spacing) [-]
P.cam.blinks  = [1.5 4.6];          % blink start times in the scenario [s] (150 ms each)
%  Frame source for camera_open: 'webcam' (real OV9281), 'video' (recorded file),
%  'images' (folder of PNG/JPG), 'synthetic' (rendered eyes, no hardware)
P.cam.source     = 'webcam';
P.cam.deviceName = 'openaicam';     % part of the webcam name (webcamlist shows the OV9281 as 'openaicam'); '' = first camera
P.cam.videoFile  = '';              % for 'video'
P.cam.imageDir   = '';              % for 'images'
P.cam.every      = 2;               % in run_rig: grab a frame every N control periods (25 Hz at Ts = 20 ms)
P.cam.exposure   = -6;              % manual exposure, log2 of seconds (-6 = 16 ms); [] = auto
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
%  Which eye finder run_rig track_camera uses with P.trk.source = 'both':
%   'face'  : eye_in_face.m, the iris (darkest round spot) in each half of the eye
%             band of the face box. On the rig at ~50 cm the iris is ~10 px wide and
%             the pupil is not darker than the iris, so the pupil detector misses
%             most frames (2026-11-09 recording: eyes 29-39 %; with 'face': 97 %).
%   'pupil' : eye_detect.m (dark pupil + glints, needs a close, sharp eye image)
P.det.method          = 'face';
P.det.faceBand        = [-0.12 0.10]; % eye band rows: P.face.eyeLevel + this (fraction of box height)
P.det.faceCols        = [0.15 0.85];  % columns searched (fraction of box width), split in the middle
P.det.faceIris        = 0.025;        % iris radius / face box width (~5 px at 200 px box)
P.det.faceEncl        = 2.2;          % skin must be brighter this many iris radii above and below
P.det.faceMinContrast = 0.47;         % (skin - iris)/skin; below = closed eye. Recording: open >= 0.49, closed <= 0.45
P.det.faceSep         = [0.20 0.55];  % eye spacing / box width accepted as a pair
P.det.faceGlintR      = 1.5;          % glint search disc around the iris centre (iris radii)
P.det.faceGlintK      = 0.5;          % glint: brighter than skin + this*(255 - skin) and >= P.det.glintMin

%% Eye tracker (eye_detection/eye_tracker.m)
P.trk.blinkMax    = 0.40;    % both eyes missing shorter than this = blink [s]
P.trk.lostAfter   = 1.0;     % missing longer -> 'lost', full-frame search [s]
P.trk.maxJumpM    = 0.02;    % a detection further than this from the prediction is rejected [m]
P.trk.vHeadMax    = 0.5;     % max head speed, widens the gate while an eye is unseen [m/s]

%% What the carriage follows in run_rig TASK = 'track_camera'
%  'eyes': pupil pair (needs the IR LEDs: dark pupil + glints)
%  'face': face box (Viola-Jones, Computer Vision Toolbox) -> works in room light
%          without LEDs; the pupil detector then only searches inside the face.
%  'both': face box first, pupils searched inside it; the eyes are followed while
%          both pupils are tracked, otherwise the face (live view: 'follows').
P.trk.source = 'both';
P.face.width      = 0.15;    % real width of the detected face box [m]  [CHECK: z shown vs. tape measure]
P.face.eyeLevel   = 0.40;    % eye height inside the box, from the top (fraction of box height)
P.face.scale      = 1/3;     % detect on a downscaled frame (speed): 1/2, 1/3, ... (every k-th pixel)
P.face.minSizePx  = 80;      % smallest face [px, full resolution] (~1 m away)
P.face.zAlpha     = 0.3;     % low-pass on the distance from the face width
P.face.detectFcn  = [];      % tests only: replacement detector @(img) boxes

%% Head estimate (eye_detection/head_estimate.m)
P.head.zDefault   = 0.60;    % distance used until both eyes have been seen [m]
P.head.zAlpha     = 0.2;     % low-pass on the distance from the eye spacing

%% Eye-tracking software options (to recover performance with the wide lens)
P.trk.useFilter = true;     % Kalman filter on the eye position (+ velocity)
P.trk.useVelFF  = false;    % carriage velocity feed-forward from the filter (off on the rig)
P.trk.qAcc      = 0.5;      % filter process noise: head acceleration PSD [(m/s^2)^2/Hz] (tuned)
P.trk.bSP       = 0.6;      % set-point weight in run_rig tracking + step test (1 = none; model: ~40 % overshoot)
P.trk.jumpGate  = 4;        % innovation > jumpGate*sigma + jumpMin = jump, no velocity update
P.trk.jumpMin   = 0.001;    % [m]
P.trk.vffTau    = 0.15;     % low-pass on the velocity feed-forward [s] (saccades, noise)
P.trk.deadband  = 0.01;     % carriage reference only changes if the head moved more than this [m]
P.trk.maxErr    = 0.15;     % |reference - carriage| above this for 1 s (at once in rig_move_to) = runaway -> STOP [m]
P.trk.holdBand  = 0.0003;   % motor off once the target is still and |error| < this [m] (hold_band.m)
                            %   was 3 mm until 2026-11-09: the motor then stopped 0.7-2.6 mm short
                            %   (no integral action inside the band); 0.3 mm = ~11 encoder counts
P.trk.holdExit  = 0.001;    % ... and on again when |error| > this [m] (hysteresis: no hunting)
P.trk.Vlim      = 6;        % camera tasks: motor voltage limit [V] (PSU 12 V). The 50 mm steps need < 5 V;
                            %   the Nucleo reset twice on 2026-11-10 while gaze follow drove up to 9-12 V

%% Gaze follow (gaze_estimation/gaze_follow.m, run_rig track_camera)
%  The camera moves toward where the eyes look and stops when the eyes look at it:
%  psi integrates the eye angle relative to the camera; carriage ref = x_head + z*tan(psi).
%  Start: look at the camera for P.gzf.tZero s (zeroing).
%  History 2026-11-10: "ref = x_head + z*tan(eye angle)" ran away on the rig (the camera's
%  own motion changes the apparent eye angle, ~1.9x); a start-up calibration of that
%  factor failed on the rig (R^2 < 0). The integrating form needs no calibration.
P.gzf.on        = true;     % false: follow the head only
P.gzf.xBase     = NaN;      % set by run_rig for TASK = 'track_gaze' (carriage base = rail centre)
P.gzf.tZero     = 2.0;      % zeroing: look at the camera this long [s]
P.gzf.minSamples= 10;       % frames needed for the zeroing
P.gzf.rEye      = 0.011;    % eyeball centre to iris centre [m] (eye models: ~10-11 mm)
P.gzf.Ki        = 0.8;      % camera angle speed per degree of eye angle [1/s]  [tune: higher = faster]
                            %   (1.5 on the rig 2026-11-10: camera slammed end to end at up to 9 V)
P.gzf.Bcomp     = 0.5;      % part of the camera's own effect removed (0..0.8): higher = camera travels further
P.gzf.deadDeg   = 5;        % eye angles below this are "looking at the camera" [deg]
                            %   (1 px of iris or face-box jitter is ~5 deg at 50 cm)  [tune: lower = more motion]
P.gzf.maxDeg    = 25;       % limit on psi [deg] (beyond ~30 deg the eyes are seen too much from the side)
P.gzf.mMax      = 45;       % eye angles above this are wrong iris detections (7 % of frames on the rig) [deg]
P.gzf.lostT     = 1.0;      % eyes lost longer than this [s] ...
P.gzf.lostTau   = 1.0;      % ... psi returns to 0 with this time constant [s]
P.gzf.oscDeg    = 8;        % oscillation guard: psi beyond +-this counts as one side [deg]
P.gzf.oscFlips  = 4;        % ... this many side changes ...
P.gzf.oscWin    = 4;        % ... within this time [s] -> gaze follow off

%% Servo LED aiming filter (run_rig): the eye position estimate jumps when the
%  tracker switches between eyes and face box (distance from the eye spacing vs the
%  face width), which made the LEDs twitch. Low-pass + dead band on the servo angle.
P.srv.tauAim    = 0.3;      % low-pass time constant [s]
P.srv.deadAim   = 1.5;      % ignore changes below this [deg] (LED beam +-17.5 deg)

%% Head / eye motion scenario
P.traj.Tend  = 6;
P.traj.p0    = [0; 0.05; 0.60];
P.traj.Ax    = 0.08; P.traj.fx = 0.3;
P.traj.Ay    = 0.03; P.traj.fy = 0.2;
P.traj.Az    = 0.05; P.traj.fz = 0.15;
P.traj.tJump = 3.0;  P.traj.jump = [0.05; -0.02; 0];

%% run_rig TASK = 'step' (real rig): carriage square wave around the rail centre
P.stepTest.size   = 0.05;   % [m]
P.stepTest.period = 3;      % each level held this long [s]

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
