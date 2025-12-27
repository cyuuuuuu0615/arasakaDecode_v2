package org.firstinspires.ftc.teamcode.decode;

import androidx.annotation.NonNull;

// RoadRunner imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;

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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

@Config
@Autonomous(name = "testAuto_v4_AngleFixed")
public class testAuto_v4 extends LinearOpMode {

    public enum BallColor {
        PURPLE, GREEN, NONE
    }

    public static List<BallColor> targetSequence = new ArrayList<>();
    // 陣列對應: [0]=Hole A (Top), [1]=Hole B (Mid), [2]=Hole C (Bot)
    public static BallColor[] actualBallSlots = {BallColor.NONE, BallColor.NONE, BallColor.NONE};

    private Limelight3A limelight;
    private DcMotor shooterMotor;
    private Servo angleServo; // 新增 angleServo

    @Override
    public void runOpMode() throws InterruptedException {
        // --- 硬體初始化 ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        shooterMotor = hardwareMap.get(DcMotor.class, "motor5");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0);

        // 新增 angleServo 初始化 (參考 TeleOp 設定為 REVERSE)
        angleServo = hardwareMap.get(Servo.class, "servo3");
        angleServo.setDirection(Servo.Direction.REVERSE);
        angleServo.setPosition(0.08); // 初始位置設為第一發的角度

        Servo gateServoL = hardwareMap.get(Servo.class, "servo4");
        Servo gateServoR = hardwareMap.get(Servo.class, "servo5");
        gateServoL.setDirection(Servo.Direction.REVERSE);
        gateServoR.setDirection(Servo.Direction.FORWARD);

        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10.0),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint slowAccel = new ProfileAccelConstraint(-10.0, 10.0);

        targetSequence.clear();
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);

        telemetry.addLine("Ready. Angle Servo Added & Coordinates Updated.");
        telemetry.update();

        // === Init Loop ===
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
                targetSequence.add(BallColor.GREEN);
                targetSequence.add(BallColor.PURPLE);
                targetSequence.add(BallColor.PURPLE);
                telemetry.addData("Tag", "21 (G-P-P)");
            } else if (id == 22) {
                targetSequence.clear();
                targetSequence.add(BallColor.PURPLE);
                targetSequence.add(BallColor.GREEN);
                targetSequence.add(BallColor.PURPLE);
                telemetry.addData("Tag", "22 (P-G-P)");
            } else if (id == 23) {
                targetSequence.clear();
                targetSequence.add(BallColor.PURPLE);
                targetSequence.add(BallColor.PURPLE);
                targetSequence.add(BallColor.GREEN);
                telemetry.addData("Tag", "23 (P-P-G)");
            } else {
                telemetry.addData("Tag", "Scanning... Default (P-P-P)");
            }
            telemetry.update();
        }

        waitForStart();
        limelight.stop();

        gateServoL.setPosition(0);
        gateServoR.setPosition(0);
        Arrays.fill(actualBallSlots, BallColor.NONE);

        Action startShooterPreHeat = packet -> {
            shooterMotor.setPower(0.7);
            angleServo.setPosition(0.08); // 預熱時先抬到第一發角度
            return false;
        };

        Action stopShooter = packet -> {
            shooterMotor.setPower(0);
            return false;
        };

        // === 主流程 ===
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(beginPose)
                                .afterTime(0, new Motor6Movement(hardwareMap))

                                // 第一波 (固定盲射 3 發)
                                .afterTime(0, startShooterPreHeat)
                                .strafeTo(new Vector2d(80, 0))
                                // 傳入 angleServo
                                .stopAndAdd(new SequentialShooterAction(hardwareMap, shooterMotor, angleServo))
                                .stopAndAdd(stopShooter)

                                // 去吸球
                                .strafeTo(new Vector2d(75, 24))
                                .afterTime(0, new AutoIntakeAction(hardwareMap))
                                .strafeTo(new Vector2d(75, 34), slowVel, slowAccel)

                                // 第二波 (智慧辨色射擊) - 修改座標為 (80, -14.5)
                                .afterTime(0, startShooterPreHeat)
                                .strafeTo(new Vector2d(80, -14.5))
                                // 傳入 angleServo
                                .stopAndAdd(new AutoShooterAction(hardwareMap, shooterMotor, angleServo))
                                .stopAndAdd(stopShooter)
                                .build()
                )
        );
    }

    // =========================================================
    // Action 1: AutoIntakeAction (維持不變)
    // =========================================================
    public static class AutoIntakeAction implements Action {
        private final DcMotor intakeMotor;
        private final Servo diskServo, gateServoL, gateServoR;
        private final NormalizedColorSensor colorSensor1, colorSensor2;
        private boolean initialized = false;
        private long timer = 0;
        private int currentFillStep = 0;

        private static final double FILL_POS_STEP_1 = 0.0;
        private static final double FILL_POS_STEP_2 = 0.3529;
        private static final double FILL_POS_STEP_3 = 0.7137;
        private static final double INTAKE_POWER = 1.0;
        private static final int TIME_BALL_SETTLE = 600;
        private static final int TIME_DISK_MOVE = 300;
        private static final double GATE_L_OPEN = 0.6667;
        private static final double GATE_R_OPEN = 0.6902;

        private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
        private static final float PURPLE_RATIO_LIMIT = 1.2f;

        private enum State { IDLE, WAIT_SETTLE, ROTATING, FINISHED }
        private State state = State.IDLE;

        public AutoIntakeAction(HardwareMap hardwareMap) {
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
                Arrays.fill(testAuto_v4.actualBallSlots, BallColor.NONE);
                initialized = true;
            }

            switch (state) {
                case IDLE:
                    if (currentFillStep >= 3) { state = State.FINISHED; break; }
                    BallColor detected = getDualSensorColor();
                    if (detected != BallColor.NONE) {
                        timer = System.currentTimeMillis();
                        state = State.WAIT_SETTLE;
                    }
                    break;

                case WAIT_SETTLE:
                    if (System.currentTimeMillis() - timer > TIME_BALL_SETTLE) {
                        BallColor confirmed = getDualSensorColor();
                        if (confirmed == BallColor.NONE) {
                            state = State.IDLE;
                        } else {
                            if (currentFillStep < 3) {
                                testAuto_v4.actualBallSlots[currentFillStep] = confirmed;
                            }
                            moveToNextPos();
                            timer = System.currentTimeMillis();
                            state = State.ROTATING;
                        }
                    }
                    break;

                case ROTATING:
                    if (System.currentTimeMillis() - timer > TIME_DISK_MOVE) { state = State.IDLE; }
                    break;

                case FINISHED:
                    intakeMotor.setPower(0);
                    return false;
            }

            packet.put("Intake Step", currentFillStep);
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
            return BallColor.NONE;
        }
    }

    // =========================================================
    // Action 2: AutoShooterAction (新增 angleServo 控制)
    // =========================================================
    public static class AutoShooterAction implements Action {
        private final Servo diskServo, kickerServo, gateServoL, gateServoR, angleServo;
        private final DcMotor shooterMotor;
        private boolean initialized = false;
        private long timer = 0;
        private int sequenceIndex = 0;
        private int shotCounter = 0;
        private String currentAimTarget = "";

        private static final double FIRE_POS_A = 0.8196;
        private static final double FIRE_POS_B = 0.0471;
        private static final double FIRE_POS_C = 0.4314;

        // 速度參數
        private static final double SPEED_FIRST = 0.7;
        private static final double SPEED_FOLLOW = 0.68; // 修改為 0.68

        // Angle Servo 參數
        private static final double ANGLE_FIRST = 0.08;
        private static final double ANGLE_FOLLOW = 0.03;

        private static final double KICKER_REST = 0.0;
        private static final double KICKER_SHOOT = 0.8;
        private static final double GATE_CLOSED = 0.0;
        private static final double GATE_L_OPEN = 0.6667;
        private static final double GATE_R_OPEN = 0.6902;

        private enum State { DECIDE, AIMING, KICKING, RETRACTING, STOP }
        private State state = State.DECIDE;

        // Constructor 接收 angleServo
        public AutoShooterAction(HardwareMap hardwareMap, DcMotor motor, Servo angleServo) {
            this.shooterMotor = motor;
            this.angleServo = angleServo; // 儲存參考
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
                // 初始設為第一發參數
                shooterMotor.setPower(SPEED_FIRST);
                angleServo.setPosition(ANGLE_FIRST);
                timer = System.currentTimeMillis();
                sequenceIndex = 0;
                shotCounter = 0;
                state = State.DECIDE;
                initialized = true;
            }

            switch (state) {
                case DECIDE:
                    if (sequenceIndex >= 3 || sequenceIndex >= testAuto_v4.targetSequence.size()) {
                        state = State.STOP;
                        break;
                    }

                    // --- 根據發射次數調整速度與角度 ---
                    if (shotCounter == 0) {
                        shooterMotor.setPower(SPEED_FIRST); // 0.7
                        angleServo.setPosition(ANGLE_FIRST); // 0.08
                    } else {
                        shooterMotor.setPower(SPEED_FOLLOW); // 0.68
                        angleServo.setPosition(ANGLE_FOLLOW); // 0.03
                    }

                    BallColor neededColor = testAuto_v4.targetSequence.get(sequenceIndex);
                    packet.put("Target Color", neededColor);

                    String targetSlot = findStrictSlotForColor(neededColor);

                    if (targetSlot == null) {
                        packet.put("Warning", "Color Missing, Fallback!");
                        targetSlot = findAnyOccupiedSlot();
                    }

                    if (targetSlot != null) {
                        setupShot(targetSlot);
                        state = State.AIMING;
                    } else {
                        packet.put("Error", "No balls found!");
                        sequenceIndex++;
                    }
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
                    gateServoL.setPosition(GATE_L_OPEN);
                    gateServoR.setPosition(GATE_R_OPEN);
                    diskServo.setPosition(0.0);
                    return false;
            }
            packet.put("Shots Fired", shotCounter);
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

        private String findStrictSlotForColor(BallColor color) {
            if (testAuto_v4.actualBallSlots[2] == color) return "C";
            if (testAuto_v4.actualBallSlots[1] == color) return "B";
            if (testAuto_v4.actualBallSlots[0] == color) return "A";
            return null;
        }

        private String findAnyOccupiedSlot() {
            if (testAuto_v4.actualBallSlots[2] != BallColor.NONE) return "C";
            if (testAuto_v4.actualBallSlots[1] != BallColor.NONE) return "B";
            if (testAuto_v4.actualBallSlots[0] != BallColor.NONE) return "A";
            return null;
        }

        private void removeBallFromSlot(String slot) {
            if (slot.equals("A")) testAuto_v4.actualBallSlots[0] = BallColor.NONE;
            else if (slot.equals("B")) testAuto_v4.actualBallSlots[1] = BallColor.NONE;
            else if (slot.equals("C")) testAuto_v4.actualBallSlots[2] = BallColor.NONE;
        }
    }

    // =========================================================
    // Action 3: SequentialShooterAction (新增 angleServo 控制)
    // =========================================================
    public static class SequentialShooterAction implements Action {
        private final Servo diskServo, kickerServo, gateServoL, gateServoR, angleServo;
        private final DcMotor shooterMotor;
        private boolean initialized = false;
        private long timer = 0;
        private int shotsFired = 0;

        private boolean hasBallA = true, hasBallB = true, hasBallC = true;
        private String currentTarget = "";

        private static final double FIRE_POS_A = 0.8196;
        private static final double FIRE_POS_B = 0.0471;
        private static final double FIRE_POS_C = 0.4314;

        private static final double KICKER_REST = 0.0;
        private static final double KICKER_SHOOT = 0.8;
        private static final double GATE_CLOSED = 0.0;
        private static final double GATE_L_OPEN = 0.6667;
        private static final double GATE_R_OPEN = 0.6902;

        private static final double SPEED_FIRST = 0.7;
        private static final double SPEED_FOLLOW = 0.68; // 修改為 0.68

        // Angle Servo 參數
        private static final double ANGLE_FIRST = 0.08;
        private static final double ANGLE_FOLLOW = 0.03;

        private enum State { DECIDE, AIMING, KICKING, RETRACTING, STOP }
        private State state = State.DECIDE;

        // Constructor 接收 angleServo
        public SequentialShooterAction(HardwareMap hardwareMap, DcMotor motor, Servo angleServo) {
            this.shooterMotor = motor;
            this.angleServo = angleServo; // 儲存參考
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
                // 初始設為第一發參數
                shooterMotor.setPower(SPEED_FIRST);
                angleServo.setPosition(ANGLE_FIRST);
                timer = System.currentTimeMillis();
                shotsFired = 0;
                state = State.DECIDE;
                initialized = true;
            }

            switch (state) {
                case DECIDE:
                    // --- 根據發射次數調整速度與角度 ---
                    if (shotsFired == 0) {
                        shooterMotor.setPower(SPEED_FIRST); // 0.7
                        angleServo.setPosition(ANGLE_FIRST); // 0.08
                    } else {
                        shooterMotor.setPower(SPEED_FOLLOW); // 0.68
                        angleServo.setPosition(ANGLE_FOLLOW); // 0.03
                    }

                    if (hasBallC) {
                        diskServo.setPosition(FIRE_POS_C); currentTarget = "C";
                        timer = System.currentTimeMillis(); state = State.AIMING;
                    }
                    else if (hasBallB) {
                        diskServo.setPosition(FIRE_POS_B); currentTarget = "B";
                        timer = System.currentTimeMillis(); state = State.AIMING;
                    }
                    else if (hasBallA) {
                        diskServo.setPosition(FIRE_POS_A); currentTarget = "A";
                        timer = System.currentTimeMillis(); state = State.AIMING;
                    }
                    else {
                        state = State.STOP;
                    }
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
                        if (currentTarget.equals("A")) hasBallA = false;
                        if (currentTarget.equals("B")) hasBallB = false;
                        if (currentTarget.equals("C")) hasBallC = false;
                        shotsFired++;
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
                    gateServoL.setPosition(GATE_L_OPEN);
                    gateServoR.setPosition(GATE_R_OPEN);
                    diskServo.setPosition(0.0);
                    return false;
            }
            packet.put("SeqShot", shotsFired);
            return true;
        }
    }

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
                motor6.setTargetPosition(-478);
                motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor6.setPower(1.0);
                initialized = true;
            }
            return false;
        }
    }
}