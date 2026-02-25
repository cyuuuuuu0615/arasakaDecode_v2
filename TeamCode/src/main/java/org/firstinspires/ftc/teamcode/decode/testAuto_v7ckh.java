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
@Autonomous(name = "AUTO BLUE v7.7 FULL (ROBUST)")
public class testAuto_v7ckh extends LinearOpMode {

    // --- Configuration Parameters ---
    public enum BallColor { PURPLE, GREEN, UNKNOWN, NONE }
    public static List<BallColor> targetSequence = new ArrayList<>();
    public static BallColor[] actualBallSlots = {BallColor.NONE, BallColor.NONE, BallColor.NONE};

    public static boolean isShootingMode = false;
    public static boolean isPreheating = false;
    public static boolean useBackShootingParams = false;

    public enum TurretState { MANUAL_POSITION, AUTO_TRACKING, IDLE }
    public static TurretState currentTurretState = TurretState.MANUAL_POSITION;

    public static int targetTurretPos = 0;
    public static int TARGET_TAG_ID = 20;

    // --- PID Constants (Synced with TeleOp) ---
    public static double TARGET_TX = 8.0;
    public static double BACK_SHOT_TX_OFFSET = 0.0;
    public static double TURRET_KP = 0.016;
    public static double TURRET_KD = 0.073;
    public static double MIN_POWER = 0.06;
    public static double MAX_POWER = 0.45;
    public static double DEADBAND = 1.0;

    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(90, 0, 0, 15);

    // Shooter & Servo Params
    private static final double CAMERA_HEIGHT = 14.5;
    private static final double TARGET_HEIGHT = 39.0;
    private static final double MOUNT_ANGLE = 17.8;
    private static final double RPM_BASE_FAR = 760.0;
    private static final double RPM_IDLE = 300.0;
    private static final double ANGLE_CLOSE = 0.0;
    private static final double ANGLE_FAR = 0.12;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware Initialization ---
        SharedHardware robot = new SharedHardware(hardwareMap);
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(9),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint slowAccel = new ProfileAccelConstraint(-9, 9);

        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();

        setSequence(BallColor.PURPLE, BallColor.PURPLE, BallColor.PURPLE);

        // --- INIT LOOP ---
        while (opModeInInit()) {
            LLResult result = robot.limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("LL STATUS", "OK");
                telemetry.addData("INIT TX", result.getTx());
            } else {
                telemetry.addData("LL STATUS", "WAITING FOR TARGET...");
            }
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Reset positions
        robot.angleServo.setPosition(ANGLE_CLOSE);
        robot.gateServoL.setPosition(0);
        robot.gateServoR.setPosition(0);
        robot.intakeMotor.setPower(1.0);

        // Mock starting balls for testing
        actualBallSlots[0] = BallColor.PURPLE;
        actualBallSlots[1] = BallColor.PURPLE;
        actualBallSlots[2] = BallColor.GREEN;

        // --- Define Actions ---
        BackgroundSystemAction systemAction = new BackgroundSystemAction(robot, telemetry);
        Action cmdTurretTrack = packet -> { currentTurretState = TurretState.AUTO_TRACKING; return false; };
        Action cmdTurretSmall = packet -> { currentTurretState = TurretState.MANUAL_POSITION; targetTurretPos = -43; return false; };
        Action cmdStartPreheat = packet -> { isPreheating = true; return false; };
        Action cmdStopPreheat  = packet -> { isPreheating = false; return false; };
        Action closeGate = packet -> { robot.gateServoL.setPosition(0); robot.gateServoR.setPosition(0); return false; };

        // --- Execution ---
        Actions.runBlocking(
                new ParallelAction(
                        systemAction,
                        new SequentialAction(
                                drive.actionBuilder(beginPose)
                                        // Step 1: Aim and Shoot First Set
                                        .afterTime(0, cmdTurretSmall)
                                        .afterTime(0, cmdStartPreheat)
                                        .stopAndAdd(cmdTurretTrack)
                                        .waitSeconds(1.0)
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(cmdStopPreheat)

                                        // Step 2: Move to Intake
                                        .strafeTo(new Vector2d(-24.5, -14))
                                        .afterTime(0, cmdStartPreheat)
                                        .afterTime(0, new AutoIntakeAction(robot, telemetry))
                                        .afterTime(0, cmdTurretSmall)
                                        .strafeTo(new Vector2d(-24.5, -36), slowVel, slowAccel)

                                        // Step 3: Return and Shoot
                                        .stopAndAdd(cmdTurretTrack)
                                        .strafeTo(new Vector2d(0, 0))
                                        .stopAndAdd(new BackShooterAction(robot, telemetry))
                                        .stopAndAdd(closeGate)
                                        .stopAndAdd(cmdStopPreheat)
                                        .build()
                        )
                )
        );
    }

    private void setSequence(BallColor c1, BallColor c2, BallColor c3) {
        targetSequence.clear();
        targetSequence.add(c1); targetSequence.add(c2); targetSequence.add(c3);
    }

    // --- Background System Control ---
    public static class BackgroundSystemAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private double lastError = 0;
        private double lastResultTimestamp = 0;
        private int staleCounter = 0;

        public BackgroundSystemAction(SharedHardware robot, Telemetry telemetry) {
            this.robot = robot; this.telemetry = telemetry;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            LLResult result = robot.limelight.getLatestResult();

            boolean isNewData = false;
            boolean hasTarget = false;
            double tx = 0;

            if (result != null && result.isValid()) {
                double currentTS = result.getTimestamp();
                if (currentTS != lastResultTimestamp) {
                    isNewData = true;
                    lastResultTimestamp = currentTS;
                    staleCounter = 0;

                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult tag : tags) {
                        if (tag.getFiducialId() == TARGET_TAG_ID) {
                            hasTarget = true;
                            tx = tag.getTargetXDegrees();
                            break;
                        }
                    }
                } else {
                    staleCounter++;
                }
            } else {
                staleCounter++;
            }

            // Turret Control
            if (currentTurretState == TurretState.AUTO_TRACKING) {
                if (robot.baseMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    robot.baseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                // Robust logic: only move if we have fresh target data
                if (isNewData && hasTarget) {
                    double activeTargetTx = TARGET_TX + (useBackShootingParams ? BACK_SHOT_TX_OFFSET : 0);
                    double error = tx - activeTargetTx;
                    double dTerm = (error - lastError) * TURRET_KD;

                    if (Math.abs(error) > DEADBAND) {
                        double power = (error * TURRET_KP) + dTerm;
                        power += (error > 0) ? MIN_POWER : -MIN_POWER;
                        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
                        robot.baseMotor.setPower(power);
                    } else {
                        robot.baseMotor.setPower(0);
                    }
                    lastError = error;
                }
                else if (staleCounter > 8) { // Safety: kill motor if camera freezes
                    robot.baseMotor.setPower(0);
                }
            } else if (currentTurretState == TurretState.MANUAL_POSITION) {
                if (robot.baseMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    robot.baseMotor.setTargetPosition(targetTurretPos);
                    robot.baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.baseMotor.setPower(1.0);
                } else {
                    robot.baseMotor.setTargetPosition(targetTurretPos);
                }
            }

            // Shooter Velocity
            double targetRpm = (isPreheating || isShootingMode) ? RPM_BASE_FAR : RPM_IDLE;
            robot.shooterRight.setVelocity(targetRpm);
            robot.shooterLeft.setPower(robot.shooterRight.getPower());

            // Dashboard Telemetry
            packet.put("LL Status", staleCounter > 0 ? "FROZEN/STALE" : "LIVE");
            packet.put("LL TX", hasTarget ? tx : "N/A");
            packet.put("Turret Mode", currentTurretState);

            return true;
        }
    }

    // --- Intake Action ---
    public static class AutoIntakeAction implements Action {
        private final SharedHardware robot;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int currentFillStep = 0;
        private enum State { IDLE, WAIT_SETTLE, ROTATING, FULL }
        private State state = State.IDLE;

        public AutoIntakeAction(SharedHardware robot, Telemetry telemetry) {
            this.robot = robot; this.telemetry = telemetry;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                robot.gateServoL.setPosition(0.6667);
                robot.gateServoR.setPosition(0.6902);
                robot.intakeMotor.setPower(1.0);
                initialized = true;
            }
            if (currentFillStep >= 3) return false;

            switch (state) {
                case IDLE:
                    if (ballDetected(robot)) {
                        timer = System.currentTimeMillis();
                        state = State.WAIT_SETTLE;
                    }
                    break;
                case WAIT_SETTLE:
                    if (System.currentTimeMillis() - timer > 100) {
                        moveToNextPos();
                        timer = System.currentTimeMillis();
                        state = State.ROTATING;
                    }
                    break;
                case ROTATING:
                    if (System.currentTimeMillis() - timer > 400) state = State.IDLE;
                    break;
            }
            return true;
        }

        private boolean ballDetected(SharedHardware robot) {
            return robot.colorSensor1.getNormalizedColors().alpha > 0.7 ||
                    robot.colorSensor2.getNormalizedColors().alpha > 0.7;
        }

        private void moveToNextPos() {
            double[] pos = {0.0, 0.3529, 0.7137};
            if (currentFillStep < 2) robot.diskServo.setPosition(pos[++currentFillStep]);
            else currentFillStep = 3;
        }
    }

    // --- Shooter Action ---
    public static class BackShooterAction implements Action {
        private final SharedHardware robot;
        private int shots = 0;
        private long timer = 0;
        private enum State { AIM, KICK, RESET, DONE }
        private State state = State.AIM;

        public BackShooterAction(SharedHardware robot, Telemetry telemetry) { this.robot = robot; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            isShootingMode = true;
            switch (state) {
                case AIM:
                    double[] firePos = {0.8196, 0.0471, 0.4314}; // A, B, C
                    robot.diskServo.setPosition(firePos[shots]);
                    timer = System.currentTimeMillis();
                    state = State.KICK;
                    break;
                case KICK:
                    if (System.currentTimeMillis() - timer > 800) {
                        robot.kickerServo.setPosition(0.8);
                        timer = System.currentTimeMillis();
                        state = State.RESET;
                    }
                    break;
                case RESET:
                    if (System.currentTimeMillis() - timer > 400) {
                        robot.kickerServo.setPosition(0.0);
                        shots++;
                        state = (shots >= 3) ? State.DONE : State.AIM;
                    }
                    break;
                case DONE:
                    isShootingMode = false;
                    return false;
            }
            return true;
        }
    }

    // --- Hardware Container ---
    public static class SharedHardware {
        public Limelight3A limelight;
        public DcMotorEx shooterRight, shooterLeft, baseMotor, intakeMotor;
        public Servo angleServo, diskServo, kickerServo, gateServoL, gateServoR;
        public NormalizedColorSensor colorSensor1, colorSensor2;

        public SharedHardware(HardwareMap map) {
            limelight = map.get(Limelight3A.class, "limelight");
            shooterRight = map.get(DcMotorEx.class, "motor7");
            shooterLeft = map.get(DcMotorEx.class, "motor5");
            baseMotor = map.get(DcMotorEx.class, "motor6");
            intakeMotor = map.get(DcMotorEx.class, "motor4");

            angleServo = map.get(Servo.class, "servo3");
            diskServo = map.get(Servo.class, "servo2");
            kickerServo = map.get(Servo.class, "servo1");
            gateServoL = map.get(Servo.class, "servo4");
            gateServoR = map.get(Servo.class, "servo5");

            colorSensor1 = map.get(NormalizedColorSensor.class, "colorSensor1");
            colorSensor2 = map.get(NormalizedColorSensor.class, "colorSensor2");

            baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            baseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            kickerServo.scaleRange(0.0, 0.5);
            gateServoL.setDirection(Servo.Direction.REVERSE);
        }
    }
}