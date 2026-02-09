package org.firstinspires.ftc.teamcode.decode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

@Config
@Autonomous(name = "AUTO NOW RED") // [修改] 名稱改為 RED
public class testAuto_v5 extends LinearOpMode { // [修改] Class 名稱

    // === 1. 全域狀態變數 ===
    public enum BallColor { PURPLE, GREEN, UNKNOWN, NONE }
    public static List<BallColor> targetSequence = new ArrayList<>();
    public static BallColor[] actualBallSlots = {BallColor.NONE, BallColor.NONE, BallColor.NONE};

    // === 系統狀態控制 ===
    public static boolean isShootingMode = false;
    public static boolean isPreheating = false;

    // 判斷是否使用後場修正參數
    public static boolean useBackShootingParams = false;

    // 砲塔模式控制
    public enum TurretState { MANUAL_POSITION, AUTO_TRACKING, IDLE }
    public static TurretState currentTurretState = TurretState.MANUAL_POSITION;

    public static int targetTurretPos = 0;

    // === [前場] 瞄準參數 ===
    // [修改] 藍隊是 3.0，紅隊反轉為 -3.0
    public static double TARGET_TX = -3.0;

    // === [後場] 修正參數 ===
    public static double BACK_SHOT_RPM_BOOST = 320.0;
    // [修改] 藍隊是 2.5，紅隊反轉為 -2.5
    public static double BACK_SHOT_TX_OFFSET = -2.5;

    // === [來自 Teleop] PIDF & 參數 ===
    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(92, 0, 0, 15);

    // === [來自 Teleop] 幾何參數 ===
    private static final double CAMERA_HEIGHT = 14.5;
    private static final double TARGET_HEIGHT = 39.0;
    private static final double MOUNT_ANGLE = 17.8;

    // === [來自 Teleop] RPM 參數 ===
    private static final double RPM_SLOPE_CLOSE = 11.0;
    private static final double RPM_BASE_CLOSE = 610.0;
    private static final double RPM_SLOPE_FAR = 10.0;
    private static final double RPM_BASE_FAR = 620.0;
    private static final double RPM_IDLE = 300.0;

    // === [來自 Teleop] Servo 角度 ===
    private static final double ANGLE_CLOSE = 0.0;
    private static final double ANGLE_FAR = 0.12;
    private static final double DISTANCE_THRESHOLD = 35.0;

    // === [實測更新] 砲塔 PID 與修正 ===
    private static final double TURRET_KP = 0.011;
    private static final double TURRET_KD = 0.095;
    private static final double MIN_POWER = 0.06;
    private static final double MAX_POWER = 0.40;
    private static final double DEADBAND = 1.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // === 強制重置全域變數 ===
        targetTurretPos = 0;
        isShootingMode = false;
        isPreheating = false;
        useBackShootingParams = false;
        currentTurretState = TurretState.MANUAL_POSITION;
        Arrays.fill(actualBallSlots, BallColor.NONE);

        SharedHardware robot = new SharedHardware(hardwareMap);
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // 速度限制
        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint slowAccel = new ProfileAccelConstraint(-10, 10);

        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();

        // 預設順序
        targetSequence.clear();
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);

        telemetry.addLine("Ready: AUTO NOW RED (Inverse Coords)");
        telemetry.update();

        // Init Loop
        while (opModeInInit()) {
            LLResult result = robot.limelight.getLatestResult();
            int id = -1;
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (!tags.isEmpty()) id = (int) tags.get(0).getFiducialId();
            }
            // TODO: 確認紅隊是否使用相同的 ID (21, 22, 23)。如果不一樣，請在此處修改。
            if (id == 21) setSequence(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE);
            else if (id == 22) setSequence(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE);
            else if (id == 23) setSequence(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN);

            telemetry.addData("Randomization Tag", id);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Start: 確保 Angle Servo 歸位
        robot.angleServo.setPosition(ANGLE_CLOSE);

        // [修改] 一開始就啟動 Intake 並且不再關閉
        robot.intakeMotor.setPower(1.0);

        // 設定初始持球狀態
        actualBallSlots[0] = BallColor.PURPLE;
        actualBallSlots[1] = BallColor.PURPLE;
        actualBallSlots[2] = BallColor.GREEN;

        // === Action 定義 ===
        Action cmdTurretBig = packet -> { currentTurretState = TurretState.MANUAL_POSITION; targetTurretPos = 370; return false; };
        Action cmdTurretSmall = packet -> { currentTurretState = TurretState.MANUAL_POSITION; targetTurretPos = -35; return false; };
        Action cmdTurretTrack = packet -> { currentTurretState = TurretState.AUTO_TRACKING; return false; };
        Action cmdStartPreheat = packet -> { isPreheating = true; return false; };
        Action cmdStopPreheat  = packet -> { isPreheating = false; return false; };

        // [修改] 讓這個 Action 變成空執行，不關閉 Intake
        Action cmdStopIntake = packet -> { return false; };

        // [修改] 讓這個 Action 變成空執行
        Action closeGate = packet -> { return false; };

        // === 路徑執行 (所有 Vector2d X, Y 取反) ===
        Actions.runBlocking(
                new ParallelAction(
                        // 1. 後台系統 (砲塔瞄準 + 飛輪控速)
                        new BackgroundSystemAction(robot, telemetry),

                        // 2. 主流程
                        new SequentialAction(
                                drive.actionBuilder(beginPose)
                                        // --- 第一階段：前場射擊 (Front Shot) ---
                                        .afterTime(0, cmdTurretBig)
                                        .afterTime(0, cmdStartPreheat)
                                        // Blue: (-80, 0) -> Red: (80, 0)
                                        .strafeTo(new Vector2d(80, 0))

                                        .stopAndAdd(cmdTurretTrack)
                                        .waitSeconds(0.1)
                                        .stopAndAdd(new FrontShooterAction(robot, telemetry))
                                        .stopAndAdd(cmdStopPreheat)

                                        // --- 第二階段：吸球 (V8 Style) ---
                                        // Blue: (-75, -14) -> Red: (75, 14)
                                        .strafeTo(new Vector2d(75, 14))
                                        .afterTime(0, cmdStartPreheat)

                                        .afterTime(0, new AutoIntakeAction(robot, telemetry))

                                        .afterTime(0, cmdTurretSmall)

                                        // Blue: (-76, -36) -> Red: (76, 36)
                                        .strafeTo(new Vector2d(76, 36), slowVel, slowAccel)

                                        .stopAndAdd(cmdStopIntake)
                                        .stopAndAdd(cmdTurretTrack)
                                        // Blue: (-76, 0) -> Red: (76, 0)
                                        .strafeTo(new Vector2d(76,0))

                                        // --- 第三階段：後場射擊 (Back Shot) ---
                                        .strafeTo(new Vector2d(0, 0))

                                        .waitSeconds(0.6)
                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(cmdStopPreheat)
                                        //-------------------------------------------------------------------------------
                                        // Blue: (-24, -14) -> Red: (24, 14)
                                        .strafeTo(new Vector2d(24, 14))
                                        .afterTime(0, cmdStartPreheat)

                                        .afterTime(0, new AutoIntakeAction(robot, telemetry))

                                        .afterTime(0, cmdTurretSmall)

                                        // Blue: (-24, -36) -> Red: (24, 36)
                                        .strafeTo(new Vector2d(24, 36), slowVel, slowAccel)

                                        .stopAndAdd(cmdStopIntake) // 空的
                                        .stopAndAdd(cmdTurretSmall)
                                        .stopAndAdd(cmdTurretTrack)
                                        .strafeTo(new Vector2d(0, 0))

                                        .stopAndAdd(closeGate) // 空的
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(cmdStopPreheat)

                                        //---------------------------------------------------

                                        // Blue: (-63, -12) -> Red: (63, 12)
                                        .strafeTo(new Vector2d(63, 12))


                                        .build()
                        )
                )
        );
    }

    private void setSequence(BallColor c1, BallColor c2, BallColor c3) {
        targetSequence.clear();
        targetSequence.add(c1); targetSequence.add(c2); targetSequence.add(c3);
    }

    // =========================================================
    // Shared Hardware Class (不變)
    // =========================================================
    public static class SharedHardware {
        public Limelight3A limelight;
        public DcMotorEx shooterRight, shooterLeft;
        public DcMotor baseMotor;
        public Servo angleServo, diskServo, kickerServo, gateServoL, gateServoR;
        public DcMotor intakeMotor;
        public NormalizedColorSensor colorSensor1, colorSensor2;

        public SharedHardware(HardwareMap map) {
            limelight = map.get(Limelight3A.class, "limelight");

            shooterRight = map.get(DcMotorEx.class, "motor7");
            shooterLeft = map.get(DcMotorEx.class, "motor5");

            shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
            shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            baseMotor = map.get(DcMotor.class, "motor6");
            baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            angleServo = map.get(Servo.class, "servo3"); angleServo.setDirection(Servo.Direction.REVERSE);
            diskServo = map.get(Servo.class, "servo2");
            kickerServo = map.get(Servo.class, "servo1"); kickerServo.scaleRange(0.0, 0.5); kickerServo.setPosition(0.0);

            gateServoL = map.get(Servo.class, "servo4"); gateServoL.setDirection(Servo.Direction.REVERSE);
            gateServoR = map.get(Servo.class, "servo5"); gateServoR.setDirection(Servo.Direction.FORWARD);

            intakeMotor = map.get(DcMotor.class, "motor4");
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            colorSensor1 = map.get(NormalizedColorSensor.class, "colorSensor1");
            colorSensor2 = map.get(NormalizedColorSensor.class, "colorSensor2");
            colorSensor1.setGain(25.0f); colorSensor2.setGain(25.0f);
        }
    }

    // =========================================================
    // BackgroundSystemAction
    // =========================================================
    public static class BackgroundSystemAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private double lastError = 0;
        private double currentCommandedRpm = RPM_IDLE;
        private double lastValidTargetRpm = RPM_IDLE;
        private static final double RPM_RAMP_DOWN_STEP = 0.4;

        // [重要] 請確認紅隊使用的 Tag ID。藍隊是 20，紅隊通常不同。
        // 如果您不確定，請檢查 Teleop 或場地說明。
        private static final int TARGET_TAG_ID = 24;

        public BackgroundSystemAction(SharedHardware robot, Telemetry telemetry) {
            this.robot = robot; this.telemetry = telemetry;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            LLResult result = robot.limelight.getLatestResult();
            boolean validTarget = false;
            double tx = 0, ty = 0;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : tags) {
                    if (tag.getFiducialId() == TARGET_TAG_ID) {
                        validTarget = true;
                        tx = result.getTx();
                        ty = result.getTy();
                        break;
                    }
                }
            }

            double activeTargetTx = TARGET_TX;
            if (useBackShootingParams) {
                activeTargetTx += BACK_SHOT_TX_OFFSET;
            }

            switch (currentTurretState) {
                case MANUAL_POSITION:
                    boolean modeNeedsSetting = (robot.baseMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION);
                    boolean posNeedsSetting = (robot.baseMotor.getTargetPosition() != targetTurretPos);
                    if (modeNeedsSetting) {
                        robot.baseMotor.setTargetPosition(targetTurretPos);
                        robot.baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.baseMotor.setPower(1.0);
                    } else if (posNeedsSetting) {
                        robot.baseMotor.setTargetPosition(targetTurretPos);
                    }
                    break;
                case AUTO_TRACKING:
                    if (robot.baseMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) robot.baseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (validTarget) {
                        double error = tx - activeTargetTx;
                        double dTerm = (error - lastError) * TURRET_KD;
                        double power = (error * TURRET_KP) + dTerm;
                        if (Math.abs(error) > DEADBAND) {
                            power += (error > 0 ? MIN_POWER : -MIN_POWER);
                            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
                            robot.baseMotor.setPower(power);
                        } else robot.baseMotor.setPower(0);
                        lastError = error;
                    } else robot.baseMotor.setPower(0);
                    break;
                case IDLE: robot.baseMotor.setPower(0); break;
            }

            double desiredRpm;
            double calculatedDistance = -1;
            if (isShootingMode || isPreheating) {
                if (validTarget) {
                    double angleRad = Math.toRadians(MOUNT_ANGLE + ty);
                    calculatedDistance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);
                    if (calculatedDistance <= DISTANCE_THRESHOLD) {
                        robot.angleServo.setPosition(ANGLE_CLOSE);
                        desiredRpm = (RPM_SLOPE_CLOSE * calculatedDistance) + RPM_BASE_CLOSE;
                    } else {
                        robot.angleServo.setPosition(ANGLE_FAR);
                        desiredRpm = (RPM_SLOPE_FAR * calculatedDistance) + RPM_BASE_FAR;
                    }

                    if (useBackShootingParams) {
                        desiredRpm += BACK_SHOT_RPM_BOOST;
                    }

                    desiredRpm = Math.max(0, Math.min(2800, desiredRpm));
                    lastValidTargetRpm = desiredRpm;
                } else {
                    if (lastValidTargetRpm > RPM_IDLE + 50) desiredRpm = lastValidTargetRpm;
                    else { desiredRpm = RPM_BASE_FAR; robot.angleServo.setPosition(ANGLE_FAR); }
                }
            } else desiredRpm = RPM_IDLE;

            if (desiredRpm >= currentCommandedRpm) currentCommandedRpm = desiredRpm;
            else {
                currentCommandedRpm -= RPM_RAMP_DOWN_STEP;
                if (currentCommandedRpm < desiredRpm) currentCommandedRpm = desiredRpm;
            }

            robot.shooterRight.setVelocity(currentCommandedRpm);
            robot.shooterLeft.setPower(robot.shooterRight.getPower());

            packet.put("RPM Command", currentCommandedRpm);
            packet.put("Target TX", activeTargetTx);
            packet.put("Current TX", tx);
            return true;
        }
    }

    // =========================================================
    // AutoIntakeAction
    // =========================================================
    public static class AutoIntakeAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int currentFillStep = 0;
        private enum State { INIT, IDLE, WAIT_SETTLE, ROTATING, FULL, DONE }
        private State state = State.INIT;
        private static final double FILL_POS_1 = 0.0;
        private static final double FILL_POS_2 = 0.3529;
        private static final double FILL_POS_3 = 0.7137;
        private static final long TIME_BALL_SETTLE = 50;
        private static final long TIME_DISK_MOVE = 100;
        private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
        private static final float PURPLE_RATIO_LIMIT = 1.2f;

        public AutoIntakeAction(SharedHardware robot, Telemetry telemetry) {
            this.robot = robot;
            this.telemetry = telemetry;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                robot.diskServo.setPosition(FILL_POS_1);
                robot.intakeMotor.setPower(1.0);
                Arrays.fill(actualBallSlots, BallColor.NONE);
                currentFillStep = 0;
                state = State.IDLE;
                initialized = true;
            }
            if (currentFillStep >= 3) state = State.FULL;

            switch (state) {
                case IDLE:
                    BallColor detected = getSensorColor();
                    if (detected != BallColor.NONE) { timer = System.currentTimeMillis(); state = State.WAIT_SETTLE; }
                    break;
                case WAIT_SETTLE:
                    if (System.currentTimeMillis() - timer > TIME_BALL_SETTLE) {
                        BallColor confirmed = getSensorColor();
                        if (confirmed != BallColor.NONE) {
                            actualBallSlots[currentFillStep] = confirmed;
                            moveToNextPos();
                            timer = System.currentTimeMillis();
                            state = State.ROTATING;
                        } else state = State.IDLE;
                    }
                    break;
                case ROTATING:
                    if (System.currentTimeMillis() - timer > TIME_DISK_MOVE) state = State.IDLE;
                    break;
                case FULL:
                    return false;
            }
            return true;
        }

        private void moveToNextPos() {
            if (currentFillStep == 0) { robot.diskServo.setPosition(FILL_POS_2); currentFillStep = 1; }
            else if (currentFillStep == 1) { robot.diskServo.setPosition(FILL_POS_3); currentFillStep = 2; }
            else currentFillStep = 3;
        }
        private BallColor getSensorColor() {
            BallColor c1 = checkSensor(robot.colorSensor1);
            if (c1 != BallColor.NONE) return c1;
            return checkSensor(robot.colorSensor2);
        }
        private BallColor checkSensor(NormalizedColorSensor sensor) {
            NormalizedRGBA color = sensor.getNormalizedColors();
            if (color.alpha < MIN_DETECT_BRIGHTNESS) return BallColor.NONE;
            if (color.blue > color.green && color.blue > color.red) {
                if (color.blue > (color.green * PURPLE_RATIO_LIMIT)) return BallColor.PURPLE;
            }
            if (color.green > color.red) {
                if (color.green >= color.blue || (color.green > color.blue * 0.85f)) return BallColor.GREEN;
            }
            return BallColor.NONE;
        }
    }

    // =========================================================
    // Action 1: FrontShooterAction
    // =========================================================
    public static class FrontShooterAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int shotsFired = 0;
        private int sequenceIndex = 0;
        private String currentSlot = "";
        private static final double FIRE_POS_A = 0.8196;
        private static final double FIRE_POS_B = 0.0471;
        private static final double FIRE_POS_C = 0.4314;
        private enum State { INIT, CHECK_RPM, AIM_DISK, KICK, RETRACT, DONE }
        private State state = State.INIT;

        public FrontShooterAction(SharedHardware robot, Telemetry telemetry) { this.robot = robot; this.telemetry = telemetry; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isShootingMode = true;
            useBackShootingParams = false;

            if (!initialized) {
                shotsFired = 0;
                sequenceIndex = 0;
                state = State.INIT;
                initialized = true;
            }
            switch (state) {
                case INIT:
                    if (shotsFired >= 3 || sequenceIndex >= targetSequence.size()) state = State.DONE;
                    else { state = State.CHECK_RPM; timer = System.currentTimeMillis(); }
                    break;
                case CHECK_RPM:
                    if (System.currentTimeMillis() - timer > 100) {
                        state = State.AIM_DISK;
                        BallColor needed = targetSequence.get(sequenceIndex);
                        String slot = findSlotForColor(needed);
                        if (slot == null) slot = findAnyOccupiedSlot();
                        if (slot == null) { state = State.DONE; break; }
                        currentSlot = slot; setupDisk(slot); timer = System.currentTimeMillis();
                    }
                    break;
                case AIM_DISK:
                    if (System.currentTimeMillis() - timer > 550) {
                        robot.kickerServo.setPosition(0.8);
                        timer = System.currentTimeMillis();
                        state = State.KICK;
                    }
                    break;
                case KICK:
                    if (System.currentTimeMillis() - timer > 300) { robot.kickerServo.setPosition(0.0); removeBall(currentSlot); shotsFired++; sequenceIndex++; state = State.RETRACT; timer = System.currentTimeMillis(); }
                    break;
                case RETRACT:
                    if (System.currentTimeMillis() - timer > 200) state = State.INIT;
                    break;
                case DONE:
                    isShootingMode = false; robot.diskServo.setPosition(0.0); return false;
            }
            return true;
        }
        private void setupDisk(String slot) { if (slot.equals("A")) robot.diskServo.setPosition(FIRE_POS_A); else if (slot.equals("B")) robot.diskServo.setPosition(FIRE_POS_B); else robot.diskServo.setPosition(FIRE_POS_C); }
        private String findSlotForColor(BallColor color) { if (actualBallSlots[0] == color) return "A"; if (actualBallSlots[1] == color) return "B"; if (actualBallSlots[2] == color) return "C"; return null; }
        private String findAnyOccupiedSlot() { if (actualBallSlots[2] != BallColor.NONE) return "C"; if (actualBallSlots[1] != BallColor.NONE) return "B"; if (actualBallSlots[0] != BallColor.NONE) return "A"; return null; }
        private void removeBall(String slot) { if (slot.equals("A")) actualBallSlots[0] = BallColor.NONE; else if (slot.equals("B")) actualBallSlots[1] = BallColor.NONE; else actualBallSlots[2] = BallColor.NONE; }
    }

    // =========================================================
    // Action 2: BackShooterAction
    // =========================================================
    public static class BackShooterAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int shotsFired = 0;
        private int sequenceIndex = 0;
        private String currentSlot = "";

        private static final int WAIT_AIM_TIME = 600;
        private static final double FIRE_POS_A = 0.8196;
        private static final double FIRE_POS_B = 0.0471;
        private static final double FIRE_POS_C = 0.4314;
        private enum State { INIT, CHECK_RPM, AIM_DISK, KICK, RETRACT, DONE }
        private State state = State.INIT;

        public BackShooterAction(SharedHardware robot, Telemetry telemetry) { this.robot = robot; this.telemetry = telemetry; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isShootingMode = true;
            useBackShootingParams = true;

            if (!initialized) {
                shotsFired = 0;
                sequenceIndex = 0;
                state = State.INIT;
                initialized = true;
            }
            switch (state) {
                case INIT:
                    if (shotsFired >= 3 || sequenceIndex >= targetSequence.size()) state = State.DONE;
                    else { state = State.CHECK_RPM; timer = System.currentTimeMillis(); }
                    break;
                case CHECK_RPM:
                    if (System.currentTimeMillis() - timer > 150) {
                        state = State.AIM_DISK;
                        BallColor needed = targetSequence.get(sequenceIndex);
                        String slot = findSlotForColor(needed);
                        if (slot == null) slot = findAnyOccupiedSlot();
                        if (slot == null) { state = State.DONE; break; }
                        currentSlot = slot; setupDisk(slot); timer = System.currentTimeMillis();
                    }
                    break;
                case AIM_DISK:
                    if (System.currentTimeMillis() - timer > WAIT_AIM_TIME) {
                        robot.kickerServo.setPosition(0.8);
                        timer = System.currentTimeMillis();
                        state = State.KICK;
                    }
                    break;
                case KICK:
                    if (System.currentTimeMillis() - timer > 300) { robot.kickerServo.setPosition(0.0); removeBall(currentSlot); shotsFired++; sequenceIndex++; state = State.RETRACT; timer = System.currentTimeMillis(); }
                    break;
                case RETRACT:
                    if (System.currentTimeMillis() - timer > 200) state = State.INIT;
                    break;
                case DONE:
                    isShootingMode = false;
                    useBackShootingParams = false;
                    robot.diskServo.setPosition(0.0);
                    return false;
            }
            return true;
        }
        private void setupDisk(String slot) { if (slot.equals("A")) robot.diskServo.setPosition(FIRE_POS_A); else if (slot.equals("B")) robot.diskServo.setPosition(FIRE_POS_B); else robot.diskServo.setPosition(FIRE_POS_C); }
        private String findSlotForColor(BallColor color) { if (actualBallSlots[0] == color) return "A"; if (actualBallSlots[1] == color) return "B"; if (actualBallSlots[2] == color) return "C"; return null; }
        private String findAnyOccupiedSlot() { if (actualBallSlots[2] != BallColor.NONE) return "C"; if (actualBallSlots[1] != BallColor.NONE) return "B"; if (actualBallSlots[0] != BallColor.NONE) return "A"; return null; }
        private void removeBall(String slot) { if (slot.equals("A")) actualBallSlots[0] = BallColor.NONE; else if (slot.equals("B")) actualBallSlots[1] = BallColor.NONE; else actualBallSlots[2] = BallColor.NONE; }
    }
}