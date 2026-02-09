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
@Autonomous(name = "testAuto_v6")
public class testAuto_v6 extends LinearOpMode {

    public enum BallColor {
        PURPLE, GREEN, UNKNOWN, NONE
    }

    // 發射目標順序 (由 Limelight 決定)
    public static List<BallColor> targetSequence = new ArrayList<>();

    // 機器人內部實際持有的球顏色狀態
    public static BallColor[] actualBallSlots = {BallColor.NONE, BallColor.NONE, BallColor.NONE};

    private Limelight3A limelight;
    private DcMotor shooterMotor;
    private Servo angleServo;

    // 定義全域角度常數
    private static final double FIXED_SHOOTER_ANGLE = 0.2;

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
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0);

        angleServo = hardwareMap.get(Servo.class, "servo3");
        angleServo.setDirection(Servo.Direction.REVERSE);
        angleServo.setPosition(FIXED_SHOOTER_ANGLE);

        Servo gateServoL = hardwareMap.get(Servo.class, "servo4");
        Servo gateServoR = hardwareMap.get(Servo.class, "servo5");
        gateServoL.setDirection(Servo.Direction.REVERSE);
        gateServoR.setDirection(Servo.Direction.FORWARD);

        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(15.0),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint slowAccel = new ProfileAccelConstraint(-15.0, 15.0);

        // 預設順序 (防止掃描失敗)
        targetSequence.clear();
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);

        // --- 2. Init Loop (Limelight 掃描) ---
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
                telemetry.addData("Tag Detected", "21 (G-P-P)");
            } else if (id == 22) {
                targetSequence.clear();
                targetSequence.add(BallColor.PURPLE); targetSequence.add(BallColor.GREEN); targetSequence.add(BallColor.PURPLE);
                telemetry.addData("Tag Detected", "22 (P-G-P)");
            } else if (id == 23) {
                targetSequence.clear();
                targetSequence.add(BallColor.PURPLE); targetSequence.add(BallColor.PURPLE); targetSequence.add(BallColor.GREEN);
                telemetry.addData("Tag Detected", "23 (P-P-G)");
            } else {
                telemetry.addData("Tag", "Scanning... Default (P-P-P)");
            }
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        angleServo.setPosition(FIXED_SHOOTER_ANGLE);
        limelight.stop();

        // --- 3. 開始運行 ---
        gateServoL.setPosition(0);
        gateServoR.setPosition(0);

        // 初始裝彈狀態：A=紫, B=紫, C=綠
        actualBallSlots[0] = BallColor.PURPLE;
        actualBallSlots[1] = BallColor.PURPLE;
        actualBallSlots[2] = BallColor.GREEN;

        Action startShooterPreHeatAfter = packet -> {
            shooterMotor.setPower(0.7);
            return false;
        };

        Action startShooterPreHeatFirst = packet -> {
            shooterMotor.setPower(0.5);
            return false;
        };

        Action stopShooter = packet -> {
            shooterMotor.setPower(0);
            return false;
        };

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(beginPose)
                                .afterTime(0, new Motor6GameBig(hardwareMap))
                                .afterTime(0, startShooterPreHeatFirst)
                                .strafeTo(new Vector2d(-80,0))
                                // 第一波發射：根據標籤順序
                                .stopAndAdd(new SequentialShooterAction(hardwareMap, shooterMotor, angleServo, telemetry))
                                .stopAndAdd(stopShooter)

                                .strafeTo(new Vector2d(-50, 22))
                                .afterTime(0, new AutoIntakeAction(hardwareMap, telemetry))
                                .strafeTo(new Vector2d(-50, 34), slowVel, slowAccel)

                                .afterTime(0, startShooterPreHeatAfter)
                                .afterTime(0, new Motor6GameSmall(hardwareMap))
                                .strafeTo(new Vector2d(-50,0))
                                .strafeTo(new Vector2d(0,-5))
                                .stopAndAdd(new AutoShooterAction(hardwareMap,shooterMotor,angleServo,telemetry))
                                .stopAndAdd(stopShooter)

                                .strafeTo(new Vector2d(-27, 22))
                                .afterTime(0, new AutoIntakeAction(hardwareMap, telemetry))
                                .strafeTo(new Vector2d(-27, 34), slowVel, slowAccel)

                                .afterTime(0, startShooterPreHeatAfter)
                                .strafeTo(new Vector2d(0, 0))
                                .stopAndAdd(new AutoShooterAction(hardwareMap,shooterMotor,angleServo,telemetry))
                                .stopAndAdd(stopShooter)

                                .strafeTo(new Vector2d(-62.5,40))
                                .afterTime(0, new Motor6GameEnd(hardwareMap))
                                .build()
                )
        );
    }

    // =========================================================
    // Action 1: SequentialShooterAction (新版：根據標籤顏色順序發射)
    // 邏輯：讀取 targetSequence，對應初始位置 (A:紫, B:紫, C:綠)
    // =========================================================
    public static class SequentialShooterAction implements Action {
        private final Servo diskServo, kickerServo, gateServoL, gateServoR, angleServo;
        private final DcMotor shooterMotor;
        private final Telemetry telemetry;
        private boolean initialized = false;
        private long timer = 0;
        private int shotCounter = 0;
        private String currentAimTarget = "";
        private double currentPower = 0.0;

        private static final double FIRE_POS_A = 0.8196; // 放置時為紫色
        private static final double FIRE_POS_B = 0.0471; // 放置時為紫色
        private static final double FIRE_POS_C = 0.4314; // 放置時為綠色

        private static final double POWER_1 = 0.5;
        private static final double POWER_2 = 0.55;
        private static final double POWER_3 = 0.55;

        private enum State { DECIDE, AIMING, KICKING, RETRACTING, STOP }
        private State state = State.DECIDE;

        public SequentialShooterAction(HardwareMap hardwareMap, DcMotor motor, Servo angleServo, Telemetry telemetry) {
            this.shooterMotor = motor;
            this.angleServo = angleServo;
            this.telemetry = telemetry;
            diskServo = hardwareMap.get(Servo.class, "servo2");
            kickerServo = hardwareMap.get(Servo.class, "servo1");
            gateServoL = hardwareMap.get(Servo.class, "servo4");
            gateServoR = hardwareMap.get(Servo.class, "servo5");
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                gateServoL.setPosition(0.0);
                gateServoR.setPosition(0.0);
                shotCounter = 0;
                currentPower = POWER_1;
                angleServo.setPosition(FIXED_SHOOTER_ANGLE);
                state = State.DECIDE;
                initialized = true;
            }

            shooterMotor.setPower(currentPower);

            switch (state) {
                case DECIDE:
                    if (shotCounter >= 3 || shotCounter >= targetSequence.size()) {
                        state = State.STOP;
                        break;
                    }

                    // 1. 決定當前顏色目標
                    BallColor neededColor = targetSequence.get(shotCounter);
                    String bestSlot = "";

                    // 2. 根據顏色找球 (已知 A:紫, B:紫, C:綠)
                    if (neededColor == BallColor.GREEN) {
                        bestSlot = "C";
                    } else if (neededColor == BallColor.PURPLE) {
                        // 如果需要紫色，優先檢查 B 孔（距離近），再檢查 A 孔
                        if (actualBallSlots[1] == BallColor.PURPLE) bestSlot = "B";
                        else bestSlot = "A";
                    }

                    // 3. 設定動力
                    if (shotCounter == 0) currentPower = POWER_1;
                    else if (shotCounter == 1) currentPower = POWER_2;
                    else currentPower = POWER_3;

                    setupShot(bestSlot);
                    state = State.AIMING;
                    break;

                case AIMING:
                    if (System.currentTimeMillis() - timer > 800) {
                        kickerServo.setPosition(0.8); // KICKER_SHOOT
                        timer = System.currentTimeMillis();
                        state = State.KICKING;
                    }
                    break;

                case KICKING:
                    if (System.currentTimeMillis() - timer > 300) {
                        kickerServo.setPosition(0.0); // KICKER_REST
                        removeBallFromSlot(currentAimTarget);
                        shotCounter++;
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
                    shooterMotor.setPower(0);
                    return false;
            }
            return true;
        }

        private void setupShot(String slot) {
            currentAimTarget = slot;
            if (slot.equals("A")) diskServo.setPosition(FIRE_POS_A);
            else if (slot.equals("B")) diskServo.setPosition(FIRE_POS_B);
            else if (slot.equals("C")) diskServo.setPosition(FIRE_POS_C);
            timer = System.currentTimeMillis();
        }

        private void removeBallFromSlot(String slot) {
            if (slot.equals("A")) actualBallSlots[0] = BallColor.NONE;
            else if (slot.equals("B")) actualBallSlots[1] = BallColor.NONE;
            else if (slot.equals("C")) actualBallSlots[2] = BallColor.NONE;
        }
    }

    // =========================================================
    // Action 2: AutoIntakeAction (吸球並辨識顏色)
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
        private enum State { IDLE, WAIT_SETTLE, ROTATING, FINISHED }
        private State state = State.IDLE;
        private BallColor tempDetectedColor = BallColor.NONE;

        public AutoIntakeAction(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
            diskServo = hardwareMap.get(Servo.class, "servo2");
            gateServoL = hardwareMap.get(Servo.class, "servo4");
            gateServoR = hardwareMap.get(Servo.class, "servo5");
            colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
            colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
            colorSensor1.setGain(25.0f); colorSensor2.setGain(25.0f);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                gateServoL.setPosition(0.6667); // OPEN
                gateServoR.setPosition(0.6902); // OPEN
                diskServo.setPosition(FILL_POS_STEP_1);
                intakeMotor.setPower(1.0);
                currentFillStep = 0;
                Arrays.fill(actualBallSlots, BallColor.NONE);
                initialized = true;
            }

            BallColor instantReading = getDualSensorColor();

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
                    if (System.currentTimeMillis() - timer > 150) {
                        if (instantReading != BallColor.NONE) {
                            actualBallSlots[currentFillStep] = instantReading;
                            moveToNextPos();
                            timer = System.currentTimeMillis();
                            state = State.ROTATING;
                        } else { state = State.IDLE; }
                    }
                    break;
                case ROTATING:
                    if (System.currentTimeMillis() - timer > 250) state = State.IDLE;
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
            return (c1 != BallColor.NONE) ? c1 : getDetectedColor(colorSensor2);
        }

        private BallColor getDetectedColor(NormalizedColorSensor sensor) {
            NormalizedRGBA color = sensor.getNormalizedColors();
            if (color.alpha < 0.7f) return BallColor.NONE;
            if (color.blue > color.green && color.blue > (color.green * 1.2f)) return BallColor.PURPLE;
            if (color.green > color.red && (color.green >= color.blue || color.green > color.blue * 0.85f)) return BallColor.GREEN;
            return BallColor.UNKNOWN;
        }
    }

    // =========================================================
    // Action 3: AutoShooterAction (根據吸到的顏色自動找球發射)
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
        private double currentPower = 0.0;

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
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                gateServoL.setPosition(0.0); gateServoR.setPosition(0.0);
                shotCounter = 0; sequenceIndex = 0;
                currentPower = 0.65;
                angleServo.setPosition(FIXED_SHOOTER_ANGLE);
                state = State.DECIDE;
                initialized = true;
            }

            shooterMotor.setPower(currentPower);

            switch (state) {
                case DECIDE:
                    if (sequenceIndex >= 3 || sequenceIndex >= targetSequence.size()) {
                        state = State.STOP;
                        break;
                    }

                    if (shotCounter == 0) currentPower = 0.65;
                    else if (shotCounter == 1) currentPower = 0.7;
                    else currentPower = 0.75;

                    BallColor neededColor = targetSequence.get(sequenceIndex);
                    String bestSlot = findSlotForColor(neededColor);

                    if (bestSlot == null) bestSlot = findAnyOccupiedSlot();

                    setupShot(bestSlot);
                    state = State.AIMING;
                    break;

                case AIMING:
                    if (System.currentTimeMillis() - timer > 800) {
                        kickerServo.setPosition(0.8);
                        timer = System.currentTimeMillis();
                        state = State.KICKING;
                    }
                    break;
                case KICKING:
                    if (System.currentTimeMillis() - timer > 300) {
                        kickerServo.setPosition(0.0);
                        removeBallFromSlot(currentAimTarget);
                        shotCounter++; sequenceIndex++;
                        timer = System.currentTimeMillis();
                        state = State.RETRACTING;
                    }
                    break;
                case RETRACTING:
                    if (System.currentTimeMillis() - timer > 250) state = State.DECIDE;
                    break;
                case STOP:
                    shooterMotor.setPower(0);
                    gateServoL.setPosition(0.6667); gateServoR.setPosition(0.6902);
                    return false;
            }
            return true;
        }

        private void setupShot(String slot) {
            currentAimTarget = slot;
            if (slot.equals("A")) diskServo.setPosition(0.8196);
            else if (slot.equals("B")) diskServo.setPosition(0.0471);
            else if (slot.equals("C")) diskServo.setPosition(0.4314);
            timer = System.currentTimeMillis();
        }

        private String findSlotForColor(BallColor targetColor) {
            if (actualBallSlots[0] == targetColor) return "A";
            if (actualBallSlots[1] == targetColor) return "B";
            if (actualBallSlots[2] == targetColor) return "C";
            return null;
        }

        private String findAnyOccupiedSlot() {
            if (actualBallSlots[2] != BallColor.NONE) return "C";
            if (actualBallSlots[1] != BallColor.NONE) return "B";
            if (actualBallSlots[0] != BallColor.NONE) return "A";
            return "C"; // Default
        }

        private void removeBallFromSlot(String slot) {
            if (slot.equals("A")) actualBallSlots[0] = BallColor.NONE;
            else if (slot.equals("B")) actualBallSlots[1] = BallColor.NONE;
            else if (slot.equals("C")) actualBallSlots[2] = BallColor.NONE;
        }
    }

    // =========================================================
    // 手臂動作類別 (Motor6)
    // =========================================================
    public static class Motor6GameSmall implements Action {
        private final DcMotor motor6;
        private boolean initialized = false;
        public Motor6GameSmall(HardwareMap hardwareMap) {
            motor6 = hardwareMap.get(DcMotor.class, "motor6");
            motor6.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor6.setTargetPosition(22);
                motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor6.setPower(1.0);
                initialized = true;
            }
            return motor6.isBusy();
        }
    }

    public static class Motor6GameBig implements Action {
        private final DcMotor motor6;
        private boolean initialized = false;
        public Motor6GameBig(HardwareMap hardwareMap) {
            motor6 = hardwareMap.get(DcMotor.class, "motor6");
            motor6.setDirection(DcMotorSimple.Direction.REVERSE);
            motor6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor6.setTargetPosition(58);
                motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor6.setPower(1.0);
                initialized = true;
            }
            return motor6.isBusy();
        }
    }

    public static class Motor6GameEnd implements Action {
        private final DcMotor motor6;
        private boolean initialized = false;
        public Motor6GameEnd(HardwareMap hardwareMap) {
            motor6 = hardwareMap.get(DcMotor.class, "motor6");
            motor6.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor6.setTargetPosition(-201);
                motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor6.setPower(1.0);
                initialized = true;
            }
            return motor6.isBusy();
        }
    }
}