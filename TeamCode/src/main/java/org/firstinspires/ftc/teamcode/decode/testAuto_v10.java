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
@Autonomous(name = "AUTO RED")
public class testAuto_v10 extends LinearOpMode {

    public enum BallColor { PURPLE, GREEN, UNKNOWN, NONE }
    public static List<BallColor> targetSequence = new ArrayList<>();
    public static BallColor[] actualBallSlots = {BallColor.NONE, BallColor.NONE, BallColor.NONE};



    // 1. 前半場 (Front) 參數 - 適用於近距離射擊
    public static int FRONT_SHOOT_TURRET_POS = 0;  // 前半場砲塔位置 (Encoder Ticks)
    public static double FRONT_SHOOT_RPM = 560;     // 前半場發射轉速

    // 2. 後半場 (Back) 參數 - 適用於遠距離射擊
    public static int BACK_SHOOT_TURRET_POS = 0;   // 後半場砲塔位置 (Encoder Ticks)
    public static double BACK_SHOOT_RPM = 1500;      // 後半場發射轉速

    // =================================================================

    public static boolean isShootingMode = false;
    public static boolean isPreheating = false;

    // 這個變量決定我們是用 Front 參數還是 Back 參數
    public static boolean useBackShootingParams = false;

    // 我們不再需要 AUTO_TRACKING，固定位置模式即可
    public enum TurretState { MANUAL_POSITION, IDLE }
    public static TurretState currentTurretState = TurretState.MANUAL_POSITION;

    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(90, 0, 0, 15);

    // 固定角度設定
    private static final double ANGLE_CLOSE = 0.12; // 適合前半場的仰角
    private static final double ANGLE_FAR = 0.12;  // 適合後半場的仰角

    private static final double RPM_IDLE = 300.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化狀態
        isShootingMode = false;
        isPreheating = false;
        useBackShootingParams = false;
        currentTurretState = TurretState.MANUAL_POSITION;
        Arrays.fill(actualBallSlots, BallColor.NONE);

        SharedHardware robot = new SharedHardware(hardwareMap);
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(9),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint slowAccel = new ProfileAccelConstraint(-9, 9);

        // Limelight 初始化 (僅用於開局檢測)
        robot.limelight.pipelineSwitch(8);
        robot.limelight.start();

        targetSequence.clear();
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);

        telemetry.addLine("Ready: AUTO FIXED (No-Limelight Aiming)");
        telemetry.addLine("Use Dashboard to tune FRONT/BACK RPM & POS");
        telemetry.update();

        // Init Loop: 檢測隨機化 Tag (21, 22, 23)
        while (opModeInInit()) {
            LLResult result = robot.limelight.getLatestResult();
            int id = -1;
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (!tags.isEmpty()) id = (int) tags.get(0).getFiducialId();
            }
            if (id == 21) setSequence(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE);
            else if (id == 22) setSequence(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE);
            else if (id == 23) setSequence(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN);

            telemetry.addData("Randomization Tag", id);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;


        robot.angleServo.setPosition(ANGLE_CLOSE);
        robot.gateServoL.setPosition(0);
        robot.gateServoR.setPosition(0);
        robot.intakeMotor.setPower(1.0);

        // 模擬預裝球
        actualBallSlots[0] = BallColor.PURPLE;
        actualBallSlots[1] = BallColor.PURPLE;
        actualBallSlots[2] = BallColor.GREEN;

        // ====================================================================
        // 定義 Action
        // ====================================================================

        // [關鍵 Action 1] 準備前半場：設定變數為 FALSE (Front) 並開啟預熱
        // 效果：砲塔轉向 FRONT_SHOOT_TURRET_POS，飛輪加速至 FRONT_SHOOT_RPM
        Action cmdPrepareFront = packet -> {
            useBackShootingParams = false;
            isPreheating = true;
            currentTurretState = TurretState.MANUAL_POSITION;
            return false;
        };

        // [關鍵 Action 2] 準備後半場：設定變數為 TRUE (Back) 並開啟預熱
        // 效果：砲塔轉向 BACK_SHOOT_TURRET_POS，飛輪加速至 BACK_SHOOT_RPM
        Action cmdPrepareBack = packet -> {
            useBackShootingParams = true;
            isPreheating = true;
            currentTurretState = TurretState.MANUAL_POSITION;
            return false;
        };

        Action cmdStopPreheat  = packet -> { isPreheating = false; return false; };
        Action cmdStopIntake = packet -> { return false; };

        Action closeGate = packet -> {
            robot.gateServoL.setPosition(0);
            robot.gateServoR.setPosition(0);
            return false;
        };

        Actions.runBlocking(
                new ParallelAction(
                        // 1. 後台系統 (持續運行，負責維持 RPM 和 砲塔角度)
                        new BackgroundSystemAction(robot, telemetry),

                        // 2. 主流程
                        new SequentialAction(
                                drive.actionBuilder(beginPose)
                                        // === 第一波射擊 (前半場) ===
                                        .afterTime(0, cmdPrepareBack)

                                        .waitSeconds(2) // 等待到位
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(cmdStopPreheat)

                                        .strafeTo(new Vector2d(24.5, 14))


                                        .afterTime(0, new AutoIntakeAction(robot, telemetry))
                                        .afterTime(0, cmdPrepareBack)
                                        .strafeTo(new Vector2d(24.5, 36), slowVel, slowAccel)

                                        .strafeTo(new Vector2d(0, 0))

                                        .stopAndAdd(closeGate)
                                        .waitSeconds(0.5)
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(cmdStopPreheat)

                                        .strafeTo(new Vector2d(50, 14))

                                        .afterTime(0, new AutoIntakeAction(robot, telemetry))


                                        .afterTime(0, cmdPrepareBack)

                                        .strafeTo(new Vector2d(50, 36), slowVel, slowAccel)
                                        .stopAndAdd(cmdStopIntake)

                                        .strafeTo(new Vector2d(0, 0))

                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(cmdStopPreheat)

                                        .strafeTo(new Vector2d(63,36))
                                        .build()
                        )
                )
        );
    }

    private void setSequence(BallColor c1, BallColor c2, BallColor c3) {
        targetSequence.clear();
        targetSequence.add(c1); targetSequence.add(c2); targetSequence.add(c3);
    }

    // =================================================================
    // 硬體類別 (Hardware Map)
    // =================================================================
    public static class SharedHardware {
        public Limelight3A limelight;
        public DcMotorEx shooterRight, shooterLeft;
        public DcMotor baseMotor; // Turret
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

    // =================================================================
    // 後台系統 (Background System)
    // 負責：根據當前模式 (Front/Back) 控制砲塔位置和飛輪速度
    // =================================================================
    public static class BackgroundSystemAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private double currentCommandedRpm = RPM_IDLE;
        private static final double RPM_RAMP_DOWN_STEP = 0.4;

        public BackgroundSystemAction(SharedHardware robot, Telemetry telemetry) {
            this.robot = robot; this.telemetry = telemetry;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // 1. 決定目標位置和速度
            int targetPos;
            double targetRpm;
            double targetAngleServo;

            if (useBackShootingParams) {
                // 後半場模式
                targetPos = BACK_SHOOT_TURRET_POS;
                targetRpm = BACK_SHOOT_RPM;
                targetAngleServo = ANGLE_FAR;
            } else {
                // 前半場模式
                targetPos = FRONT_SHOOT_TURRET_POS;
                targetRpm = FRONT_SHOOT_RPM;
                targetAngleServo = ANGLE_CLOSE;
            }

            // 2. 控制砲塔 (Motor 6)
            if (currentTurretState == TurretState.MANUAL_POSITION) {
                if (robot.baseMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    robot.baseMotor.setTargetPosition(targetPos);
                    robot.baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.baseMotor.setPower(1.0);
                } else {
                    if (robot.baseMotor.getTargetPosition() != targetPos) {
                        robot.baseMotor.setTargetPosition(targetPos);
                    }
                    robot.baseMotor.setPower(1.0);
                }
            } else {
                robot.baseMotor.setPower(0);
            }

            // 3. 控制飛輪 (Motor 7 & 5)
            double desiredRpm;
            if (isShootingMode || isPreheating) {
                desiredRpm = Math.max(0, Math.min(2800, targetRpm));
                robot.angleServo.setPosition(targetAngleServo);
            } else {
                desiredRpm = RPM_IDLE;
            }

            // 簡單的減速緩衝
            if (desiredRpm >= currentCommandedRpm) {
                currentCommandedRpm = desiredRpm;
            } else {
                currentCommandedRpm -= RPM_RAMP_DOWN_STEP;
                if (currentCommandedRpm < desiredRpm) currentCommandedRpm = desiredRpm;
            }

            // 應用 PIDF 速度控制
            robot.shooterRight.setVelocity(currentCommandedRpm);
            robot.shooterLeft.setPower(robot.shooterRight.getPower());

            // Telemetry
            packet.put("Mode", useBackShootingParams ? "BACK" : "FRONT");
            packet.put("CMD RPM", currentCommandedRpm);
            packet.put("Turret Pos", robot.baseMotor.getCurrentPosition());
            packet.put("Turret Target", targetPos);

            return true;
        }
    }

    // =================================================================
    // 自動吸球 Action (Auto Intake) - 保持不變
    // =================================================================
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
        private static final long TIME_BALL_SETTLE = 30;
        private static final long TIME_DISK_MOVE = 60;
        private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
        private static final float PURPLE_RATIO_LIMIT = 1.2f;

        public AutoIntakeAction(SharedHardware robot, Telemetry telemetry) {
            this.robot = robot;
            this.telemetry = telemetry;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                robot.gateServoL.setPosition(0.6667);
                robot.gateServoR.setPosition(0.6902);
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

    // =================================================================
    // 前半場發射 Action (Front Shooter)
    // =================================================================
    public static class FrontShooterAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int shotsFired = 0;
        private int sequenceIndex = 0;
        private String currentSlot = "";

        // 動作時間 (ms)
        private static final double FIRE_POS_A = 0.8196;
        private static final double FIRE_POS_B = 0.0471;
        private static final double FIRE_POS_C = 0.4314;
        private enum State { INIT, CHECK_RPM, AIM_DISK, KICK, RETRACT, DONE }
        private State state = State.INIT;

        public FrontShooterAction(SharedHardware robot, Telemetry telemetry) { this.robot = robot; this.telemetry = telemetry; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isShootingMode = true;
            useBackShootingParams = false; // 強制使用前半場參數

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
                    // 這裡只等待一小段時間讓 PIDF 穩定
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
                    if (System.currentTimeMillis() - timer > 400) {
                        robot.kickerServo.setPosition(0.8);
                        timer = System.currentTimeMillis();
                        state = State.KICK;
                    }
                    break;
                case KICK:
                    if (System.currentTimeMillis() - timer > 250) {
                        robot.kickerServo.setPosition(0.0);
                        removeBall(currentSlot);
                        shotsFired++; sequenceIndex++;
                        state = State.RETRACT; timer = System.currentTimeMillis();
                    }
                    break;
                case RETRACT:
                    if (System.currentTimeMillis() - timer > 150) state = State.INIT;
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

    // =================================================================
    // 後半場發射 Action (Back Shooter)
    // =================================================================
    public static class BackShooterAction implements Action {
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

        public BackShooterAction(SharedHardware robot, Telemetry telemetry) { this.robot = robot; this.telemetry = telemetry; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isShootingMode = true;
            useBackShootingParams = true; // 強制使用後半場參數

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
                    if (System.currentTimeMillis() - timer > 400) {
                        robot.kickerServo.setPosition(0.8);
                        timer = System.currentTimeMillis();
                        state = State.KICK;
                    }
                    break;
                case KICK:
                    if (System.currentTimeMillis() - timer > 250) {
                        robot.kickerServo.setPosition(0.0);
                        removeBall(currentSlot);
                        shotsFired++; sequenceIndex++;
                        state = State.RETRACT; timer = System.currentTimeMillis();
                    }
                    break;
                case RETRACT:
                    if (System.currentTimeMillis() - timer > 150) state = State.INIT;
                    break;
                case DONE:
                    isShootingMode = false;
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