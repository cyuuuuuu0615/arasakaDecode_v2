package org.firstinspires.ftc.teamcode.decode;

import androidx.annotation.NonNull;

// RoadRunner imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

// Hardware imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

@Config
@Autonomous(name = "testAuto_buleAlliance")
public class testAuto_v8 extends LinearOpMode {

    public enum BallColor {
        PURPLE, GREEN, UNKNOWN, NONE
    }

    // 發射目標順序
    public static List<BallColor> targetSequence = new ArrayList<>();

    // 機器人內部實際持有的球顏色
    public static BallColor[] actualBallSlots = {BallColor.NONE, BallColor.NONE, BallColor.NONE};

    private Limelight3A limelight;
    private DcMotor shooterMotor;
    private Servo angleServo;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- 1. 硬體初始化 ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        shooterMotor = hardwareMap.get(DcMotor.class, "motor5");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // 讓飛輪保持慣性
        shooterMotor.setPower(0);

        angleServo = hardwareMap.get(Servo.class, "servo3");
        angleServo.setDirection(Servo.Direction.REVERSE);
        angleServo.setPosition(0.08);

        Servo gateServoL = hardwareMap.get(Servo.class, "servo4");
        Servo gateServoR = hardwareMap.get(Servo.class, "servo5");
        gateServoL.setDirection(Servo.Direction.REVERSE);
        gateServoR.setDirection(Servo.Direction.FORWARD);


        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10.0),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint slowAccel = new ProfileAccelConstraint(-10.0, 10.0);


        // 預設順序
        targetSequence.clear();
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);

        telemetry.addLine("Ready: v8 (Fix Motor & Ref updated)");
        telemetry.update();

        // --- 2. Init Loop ---
        while (opModeInInit()) {
            LLResult result = limelight.getLatestResult();
            int id = -1;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (!tags.isEmpty()) {
                    id = (int) tags.get(0).getFiducialId();
                }
            }

            if (id == 21) {
                targetSequence.clear();
                targetSequence.add(BallColor.GREEN); targetSequence.add(BallColor.PURPLE); targetSequence.add(BallColor.PURPLE);
                telemetry.addData("Tag", "21 (G-P-P)");
            } else if (id == 22) {
                targetSequence.clear();
                targetSequence.add(BallColor.PURPLE); targetSequence.add(BallColor.GREEN); targetSequence.add(BallColor.PURPLE);
                telemetry.addData("Tag", "22 (P-G-P)");
            } else if (id == 23) {
                targetSequence.clear();
                targetSequence.add(BallColor.PURPLE); targetSequence.add(BallColor.PURPLE); targetSequence.add(BallColor.GREEN);
                telemetry.addData("Tag", "23 (P-P-G)");
            } else {
                telemetry.addData("Tag", "Scanning... Default (P-P-P)");
            }
            telemetry.update();
        }

        waitForStart();
        angleServo.setPosition(0);
        limelight.stop();

        // --- 3. 開始運行 ---
        gateServoL.setPosition(0);
        gateServoR.setPosition(0);
        Arrays.fill(actualBallSlots, BallColor.NONE);

        Action startShooterPreHeat = packet -> {
            shooterMotor.setPower(0.75);
//            angleServo.setPosition(0.13);
            return false;
        };

        Action stopShooter = packet -> {
            shooterMotor.setPower(0);
            return false;
        };

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(beginPose)
//                                .afterTime(0, new Motor6Movement(hardwareMap))
//
//                                .afterTime(0, startShooterPreHeat)
//                                .strafeTo(new Vector2d(80, 0))
//
//
//                                .stopAndAdd(new SequentialShooterAction(hardwareMap, shooterMotor, angleServo, telemetry))
//                                .stopAndAdd(stopShooter)
//
//                                .strafeTo(new Vector2d(75, 23))
//                                .afterTime(0, new AutoIntakeAction(hardwareMap, telemetry))
//                                .strafeTo(new Vector2d(75, 35), slowVel, slowAccel)
//
//                                .afterTime(0, startShooterPreHeat)
//                                .strafeTo(new Vector2d(80, -14.5))
//                                .stopAndAdd(new AutoShooterAction(hardwareMap,shooterMotor,angleServo,telemetry))
//                                .stopAndAdd(stopShooter)
//
//                                .strafeTo(new Vector2d(50, 22))
//                                .afterTime(0, new AutoIntakeAction(hardwareMap, telemetry))
//                                .strafeTo(new Vector2d(50, 34), slowVel, slowAccel)
//
//                                .afterTime(0, startShooterPreHeat)
//                                .strafeTo(new Vector2d(80, -15))
//                                .stopAndAdd(new AutoShooterAction(hardwareMap,shooterMotor,angleServo,telemetry))
//                                .stopAndAdd(stopShooter)
//
//                                .strafeTo(new Vector2d(25, 22))
//                                .afterTime(0, new AutoIntakeAction(hardwareMap, telemetry))
//                                .strafeTo(new Vector2d(25, 34), slowVel, slowAccel)
//
//                                .afterTime(0, startShooterPreHeat)
//                                .strafeTo(new Vector2d(80, -16))
//                                .stopAndAdd(new AutoShooterAction(hardwareMap,shooterMotor,angleServo,telemetry))
//                                .stopAndAdd(stopShooter)
//                                .build()

//------------------------------------------------------------------------------------------------

                                .afterTime(0, new Motor6Movement(hardwareMap))

                                .afterTime(0, startShooterPreHeat)
                                .strafeTo(new Vector2d(-10,0))
                                .strafeTo(new Vector2d(0,0))



                                .stopAndAdd(new SequentialShooterAction(hardwareMap, shooterMotor, angleServo, telemetry))
                                .stopAndAdd(stopShooter)

                                .strafeTo(new Vector2d(-25, 22))
                                .afterTime(0, new AutoIntakeAction(hardwareMap, telemetry))
                                .strafeTo(new Vector2d(-25, 35), slowVel, slowAccel)

                                .afterTime(0, startShooterPreHeat)
                                .strafeTo(new Vector2d(0,0))
                                .stopAndAdd(new AutoShooterAction(hardwareMap,shooterMotor,angleServo,telemetry))
                                .stopAndAdd(stopShooter)

                                .strafeTo(new Vector2d(-50, 22))
                                .afterTime(0, new AutoIntakeAction(hardwareMap, telemetry))
                                .strafeTo(new Vector2d(-50, 34), slowVel, slowAccel)

                                .afterTime(0, startShooterPreHeat)
                                .strafeTo(new Vector2d(0, 0))
                                .stopAndAdd(new AutoShooterAction(hardwareMap,shooterMotor,angleServo,telemetry))
                                .stopAndAdd(stopShooter)

                                .strafeTo(new Vector2d(-62.5,40))

//                                .strafeTo(new Vector2d(-75, 22))
//                                .afterTime(0, new AutoIntakeAction(hardwareMap, telemetry))
//                                .strafeTo(new Vector2d(-75, 34), slowVel, slowAccel)
//
//                                .afterTime(0, startShooterPreHeat)
//                                .strafeTo(new Vector2d(0, 0))
//                                .stopAndAdd(new AutoShooterAction(hardwareMap,shooterMotor,angleServo,telemetry))
//                                .stopAndAdd(stopShooter)
                                .build()
                )
        );
    }

    // =========================================================
    // Action 1: SequentialShooterAction (已修正馬達停止問題)
    // =========================================================
    public static class SequentialShooterAction implements Action {
        private final Servo diskServo, kickerServo, gateServoL, gateServoR, angleServo;
        private final DcMotor shooterMotor;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int step = 0;
        // 新增：追蹤當前應該有的馬達速度
        private double currentPower = 0.0;
        private State state = State.PREPARE;

        private static final double FIRE_POS_A = 0.8196;
        private static final double FIRE_POS_B = 0.0471;
        private static final double FIRE_POS_C = 0.4314;
        private static final double SPEED_FIRST = 0.75;
        private static final double ANGLE_FIRST = 0.13;
        private static final double SPEED_FOLLOW = 0.8;
        private static final double ANGLE_FOLLOW = 0.13;
        private static final double KICKER_REST = 0.0;
        private static final double KICKER_SHOOT = 0.8;
        private static final double GATE_CLOSED = 0.0;
        private static final double GATE_L_OPEN = 0.6667;
        private static final double GATE_R_OPEN = 0.6902;
        private enum State { PREPARE, AIMING, KICKING, RETRACTING, STOP }

        public SequentialShooterAction(HardwareMap hardwareMap, DcMotor motor, Servo angleServo, Telemetry telemetry) {
            this.shooterMotor = motor;
            this.angleServo = angleServo;
            this.telemetry = telemetry;
            diskServo = hardwareMap.get(Servo.class, "servo2");
            kickerServo = hardwareMap.get(Servo.class, "servo1");
            gateServoL = hardwareMap.get(Servo.class, "servo4");
            gateServoR = hardwareMap.get(Servo.class, "servo5");
            gateServoL.setDirection(Servo.Direction.REVERSE);
            gateServoR.setDirection(Servo.Direction.FORWARD);
            kickerServo.setPosition(KICKER_REST);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // 初始化時設定第一段速度
                currentPower = SPEED_FIRST;
//                angleServo.setPosition(ANGLE_FIRST);
                gateServoL.setPosition(GATE_CLOSED);
                gateServoR.setPosition(GATE_CLOSED);
                step = 0;
                state = State.PREPARE;
                initialized = true;
            }

            // 【關鍵修正】每一圈都重新發送動力指令，防止馬達停止
            shooterMotor.setPower(currentPower);

            packet.put("Seq Step", step);
            packet.put("Seq State", state);
            packet.put("Shooter Power", currentPower);
            telemetry.addData("Action", "Sequential Shooter");
            telemetry.addData("Step", step);
            telemetry.addData("State", state);
            telemetry.addData("CMD Power", currentPower);
            telemetry.update();

            switch (state) {
                case PREPARE:
                    if (step > 2) { state = State.STOP; break; }
                    double targetPos = 0;
                    if (step == 0) {
                        targetPos = FIRE_POS_C;
                        currentPower = SPEED_FIRST; // 更新變數
//                        angleServo.setPosition(ANGLE_FIRST);
                    } else {
                        if (step == 1) targetPos = FIRE_POS_B;
                        else if (step == 2) targetPos = FIRE_POS_A;
                        currentPower = SPEED_FOLLOW; // 更新變數
//                        angleServo.setPosition(ANGLE_FOLLOW);
                    }
                    diskServo.setPosition(targetPos);
                    timer = System.currentTimeMillis();
                    state = State.AIMING;
                    break;
                case AIMING:
                    if (System.currentTimeMillis() - timer > 500) {
                        kickerServo.setPosition(KICKER_SHOOT);
                        timer = System.currentTimeMillis();
                        state = State.KICKING;
                    }
                    break;
                case KICKING:
                    if (System.currentTimeMillis() - timer > 300) {
                        kickerServo.setPosition(KICKER_REST);
                        step++;
                        timer = System.currentTimeMillis();
                        state = State.RETRACTING;
                    }
                    break;
                case RETRACTING:
                    if (System.currentTimeMillis() - timer > 250) {
                        state = State.PREPARE;
                    }
                    break;
                case STOP:
                    currentPower = 0; // 停止動力
                    shooterMotor.setPower(0);
                    gateServoL.setPosition(GATE_L_OPEN);
                    gateServoR.setPosition(GATE_R_OPEN);
                    diskServo.setPosition(0.0);
                    return false;
            }
            return true;
        }
    }

    // =========================================================
    // Action 2: AutoIntakeAction (保持不變)
    // =========================================================
    public static class AutoIntakeAction implements Action {
        private final DcMotor intakeMotor;
        private final Servo diskServo, gateServoL, gateServoR;
        private final NormalizedColorSensor colorSensor1, colorSensor2;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int currentFillStep = 0;

        private static final double FILL_POS_STEP_1 = 0.0;
        private static final double FILL_POS_STEP_2 = 0.3529;
        private static final double FILL_POS_STEP_3 = 0.7137;
        private static final double INTAKE_POWER = 1.0;
        private static final int TIME_BALL_SETTLE = 100;
        private static final int TIME_DISK_MOVE = 250;
        private static final double GATE_L_OPEN = 0.6667;
        private static final double GATE_R_OPEN = 0.6902;
        private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
        private static final float PURPLE_RATIO_LIMIT = 1.2f;

        private enum State { IDLE, WAIT_SETTLE, ROTATING, FINISHED }
        private State state = State.IDLE;
        private BallColor tempDetectedColor = BallColor.NONE;

        public AutoIntakeAction(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
            diskServo = hardwareMap.get(Servo.class, "servo2");
            gateServoL = hardwareMap.get(Servo.class, "servo4");
            gateServoR = hardwareMap.get(Servo.class, "servo5");
            gateServoL.setDirection(Servo.Direction.REVERSE);
            gateServoR.setDirection(Servo.Direction.FORWARD);

            colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
            colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
            colorSensor1.setGain(25.0f);
            colorSensor2.setGain(25.0f);
            if (colorSensor1 instanceof com.qualcomm.robotcore.hardware.SwitchableLight) ((com.qualcomm.robotcore.hardware.SwitchableLight)colorSensor1).enableLight(true);
            if (colorSensor2 instanceof com.qualcomm.robotcore.hardware.SwitchableLight) ((com.qualcomm.robotcore.hardware.SwitchableLight)colorSensor2).enableLight(true);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                gateServoL.setPosition(GATE_L_OPEN);
                gateServoR.setPosition(GATE_R_OPEN);
                diskServo.setPosition(FILL_POS_STEP_1);
                intakeMotor.setPower(INTAKE_POWER);
                currentFillStep = 0;
                // 注意：這裡引用了 testAuto_v8 的靜態變數
                Arrays.fill(testAuto_v8.actualBallSlots, BallColor.NONE);
                initialized = true;
            }

            BallColor instantReading = getDualSensorColor();
            packet.put("Intake Step", currentFillStep);
            telemetry.addData("Intake Step", currentFillStep);
            telemetry.addData("Color", instantReading);
            telemetry.update();

            switch (state) {
                case IDLE:
                    if (currentFillStep >= 3) { state = State.FINISHED; break; }
                    if (instantReading != BallColor.NONE) {
                        tempDetectedColor = instantReading;
                        timer = System.currentTimeMillis();
                        state = State.WAIT_SETTLE;
                    }
                    break;
                case WAIT_SETTLE:
                    if (System.currentTimeMillis() - timer > TIME_BALL_SETTLE) {
                        BallColor confirmed = instantReading;
                        if (confirmed != BallColor.NONE) {
                            if (currentFillStep < 3) {
                                if (confirmed == BallColor.UNKNOWN && tempDetectedColor != BallColor.UNKNOWN) {
                                    // 注意：這裡引用了 testAuto_v8 的靜態變數
                                    testAuto_v8.actualBallSlots[currentFillStep] = tempDetectedColor;
                                } else {
                                    testAuto_v8.actualBallSlots[currentFillStep] = confirmed;
                                }
                            }
                            moveToNextPos();
                            timer = System.currentTimeMillis();
                            state = State.ROTATING;
                        } else {
                            state = State.IDLE;
                        }
                    }
                    break;
                case ROTATING:
                    if (System.currentTimeMillis() - timer > TIME_DISK_MOVE) {
                        state = State.IDLE;
                    }
                    break;
                case FINISHED:
                    intakeMotor.setPower(0);
                    return false;
            }
            return true;
        }

        private void moveToNextPos() {
            if (currentFillStep == 0) { diskServo.setPosition(FILL_POS_STEP_2); currentFillStep = 1; }
            else if (currentFillStep == 1) { diskServo.setPosition(FILL_POS_STEP_3); currentFillStep = 2; }
            else if (currentFillStep == 2) { currentFillStep = 3; }
        }

        private BallColor getDualSensorColor() {
            BallColor c1 = getDetectedColor(colorSensor1);
            if (c1 != BallColor.NONE) return c1;
            BallColor c2 = getDetectedColor(colorSensor2);
            if (c2 != BallColor.NONE) return c2;
            return BallColor.NONE;
        }

        private BallColor getDetectedColor(NormalizedColorSensor sensor) {
            NormalizedRGBA color = sensor.getNormalizedColors();
            if (color.alpha < MIN_DETECT_BRIGHTNESS) return BallColor.NONE;
            if (color.blue > color.green && color.blue > color.red) {
                if (color.blue > (color.green * PURPLE_RATIO_LIMIT)) return BallColor.PURPLE;
            }
            if (color.green > color.red) {
                if (color.green >= color.blue || (color.green > color.blue * 0.85f)) return BallColor.GREEN;
            }
            return BallColor.UNKNOWN;
        }
    }

    // =========================================================
    // Action 3: AutoShooterAction (已修正馬達停止問題)
    // =========================================================
    public static class AutoShooterAction implements Action {
        private final Servo diskServo, kickerServo, gateServoL, gateServoR, angleServo;
        private final DcMotor shooterMotor;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int sequenceIndex = 0;
        private int shotCounter = 0;
        private String currentAimTarget = "";
        // 新增：追蹤當前應該有的馬達速度
        private double currentPower = 0.0;

        private static final double FIRE_POS_A = 0.8196;
        private static final double FIRE_POS_B = 0.0471;
        private static final double FIRE_POS_C = 0.4314;
        private static final double SPEED_FIRST = 0.75;
        private static final double ANGLE_FIRST = 0.13;
        private static final double SPEED_FOLLOW = 0.8;
        private static final double ANGLE_FOLLOW = 0.13;
        private static final double KICKER_REST = 0.0;
        private static final double KICKER_SHOOT = 0.8;
        private static final double GATE_CLOSED = 0.0;
        private static final double GATE_L_OPEN = 0.6667;
        private static final double GATE_R_OPEN = 0.6902;

        private enum State { DECIDE, AIMING, KICKING, RETRACTING, STOP }
        private State state = State.DECIDE;

        public AutoShooterAction(HardwareMap hardwareMap, DcMotor motor, Servo angleServo, Telemetry telemetry) {
            this.shooterMotor = motor;
            this.angleServo = angleServo;
            this.telemetry = telemetry;
            diskServo = hardwareMap.get(Servo.class, "servo2");
            kickerServo = hardwareMap.get(Servo.class, "servo1");
            gateServoL = hardwareMap.get(Servo.class, "servo4");
            gateServoR = hardwareMap.get(Servo.class, "servo5");
            gateServoL.setDirection(Servo.Direction.REVERSE);
            gateServoR.setDirection(Servo.Direction.FORWARD);
            kickerServo.setPosition(KICKER_REST);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                gateServoL.setPosition(GATE_CLOSED);
                gateServoR.setPosition(GATE_CLOSED);
                // 初始動力
                currentPower = SPEED_FIRST;
//                angleServo.setPosition(ANGLE_FIRST);
                timer = System.currentTimeMillis();
                sequenceIndex = 0;
                shotCounter = 0;
                state = State.DECIDE;
                initialized = true;
            }

            // 【關鍵修正】每一圈都重新發送動力指令
            shooterMotor.setPower(currentPower);

            packet.put("Smart State", state);
            packet.put("CMD Power", currentPower);
            telemetry.addData("Smart State", state);
            telemetry.addData("CMD Power", currentPower);
            telemetry.update();

            switch (state) {
                case DECIDE:
                    // 注意：這裡引用了 testAuto_v8 的靜態變數
                    if (sequenceIndex >= 3 || sequenceIndex >= testAuto_v8.targetSequence.size()) {
                        state = State.STOP;
                        break;
                    }

                    if (shotCounter == 0) {
                        currentPower = SPEED_FIRST; // 更新變數
//                        angleServo.setPosition(ANGLE_FIRST);
                    } else {
                        currentPower = SPEED_FOLLOW; // 更新變數
//                        angleServo.setPosition(ANGLE_FOLLOW);
                    }

                    // 注意：這裡引用了 testAuto_v8 的靜態變數
                    BallColor neededColor = testAuto_v8.targetSequence.get(sequenceIndex);
                    String bestSlot = findShortestPathSlotForColor(neededColor);

                    if (bestSlot == null) bestSlot = findAnyOccupiedSlot();
                    if (bestSlot == null) {
                        if (shotCounter == 0) bestSlot = "C";
                        else if (shotCounter == 1) bestSlot = "B";
                        else bestSlot = "A";
                    }

                    setupShot(bestSlot);
                    state = State.AIMING;
                    break;

                case AIMING:
                    if (System.currentTimeMillis() - timer > 600) {
                        kickerServo.setPosition(KICKER_SHOOT);
                        timer = System.currentTimeMillis();
                        state = State.KICKING;
                    }
                    break;
                case KICKING:
                    if (System.currentTimeMillis() - timer > 300) {
                        kickerServo.setPosition(KICKER_REST);
                        removeBallFromSlot(currentAimTarget);
                        shotCounter++;
                        sequenceIndex++;
                        timer = System.currentTimeMillis();
                        state = State.RETRACTING;
                    }
                    break;
                case RETRACTING:
                    if (System.currentTimeMillis() - timer > 250) {
                        state = State.DECIDE;
                    }
                    break;
                case STOP:
                    currentPower = 0; // 停止
                    shooterMotor.setPower(0);
                    gateServoL.setPosition(GATE_L_OPEN);
                    gateServoR.setPosition(GATE_R_OPEN);
                    diskServo.setPosition(0.0);
                    return false;
            }
            return true;
        }

        private void setupShot(String slot) {
            currentAimTarget = slot;
            double targetPos = 0;
            if (slot.equals("A")) targetPos = FIRE_POS_A;
            else if (slot.equals("B")) targetPos = FIRE_POS_B;
            else if (slot.equals("C")) targetPos = FIRE_POS_C;
            diskServo.setPosition(targetPos);
            timer = System.currentTimeMillis();
        }

        private String findShortestPathSlotForColor(BallColor targetColor) {
            double currentServoPos = diskServo.getPosition();
            String bestSlot = null;
            double minDistance = 1000.0;

            // 注意：這裡引用了 testAuto_v8 的靜態變數
            if (testAuto_v8.actualBallSlots[0] == targetColor) {
                double dist = Math.abs(currentServoPos - FIRE_POS_A);
                if (dist < minDistance) { minDistance = dist; bestSlot = "A"; }
            }
            if (testAuto_v8.actualBallSlots[1] == targetColor) {
                double dist = Math.abs(currentServoPos - FIRE_POS_B);
                if (dist < minDistance) { minDistance = dist; bestSlot = "B"; }
            }
            if (testAuto_v8.actualBallSlots[2] == targetColor) {
                double dist = Math.abs(currentServoPos - FIRE_POS_C);
                if (dist < minDistance) { minDistance = dist; bestSlot = "C"; }
            }
            return bestSlot;
        }

        private String findAnyOccupiedSlot() {
            // 注意：這裡引用了 testAuto_v8 的靜態變數
            if (testAuto_v8.actualBallSlots[2] != BallColor.NONE) return "C";
            if (testAuto_v8.actualBallSlots[1] != BallColor.NONE) return "B";
            if (testAuto_v8.actualBallSlots[0] != BallColor.NONE) return "A";
            return null;
        }

        private void removeBallFromSlot(String slot) {
            // 注意：這裡引用了 testAuto_v8 的靜態變數
            if (slot.equals("A")) testAuto_v8.actualBallSlots[0] = BallColor.NONE;
            else if (slot.equals("B")) testAuto_v8.actualBallSlots[1] = BallColor.NONE;
            else if (testAuto_v8.actualBallSlots.length > 2 && slot.equals("C")) testAuto_v8.actualBallSlots[2] = BallColor.NONE;
        }
    }

    // =========================================================
    // Action 4: Motor6Movement (抬手)
    // =========================================================
    public static class Motor6Movement implements Action {
        private final DcMotor motor6;
        private boolean initialized = false;
        public Motor6Movement(HardwareMap hardwareMap) {
            motor6 = hardwareMap.get(DcMotor.class, "motor6");
            motor6.setDirection(DcMotorSimple.Direction.REVERSE);
            motor6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor6.setTargetPosition(-213);
                motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor6.setPower(1.0);
                initialized = true;
            }
            return false;
        }
    }
}