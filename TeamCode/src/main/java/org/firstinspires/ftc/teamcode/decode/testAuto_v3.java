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

// Hardware imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import java.util.Arrays;

// Drive import
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "testAuto_v3")
public class testAuto_v3 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        // 1. 初始化底盤
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "motor5");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo gateServoL = hardwareMap.get(Servo.class, "servo4");
        Servo gateServoR = hardwareMap.get(Servo.class, "servo5");
        gateServoL.setDirection(Servo.Direction.REVERSE);
        gateServoR.setDirection(Servo.Direction.FORWARD);





//        // 2. 獲取並初始化 Action
//        Action intakeTask = new AutoIntakeAction(hardwareMap);
//        Action shooterTask = new AutoShooterAction(hardwareMap);

        // Define custom constraints for slow motion
// Speed: 15 inches/second (Adjust this number to be faster/slower)
        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10.0),
                new AngularVelConstraint(Math.toRadians(90))
        ));

// Acceleration: -15 to 15 (Start and stop gently)
        AccelConstraint slowAccel = new ProfileAccelConstraint(-10.0, 10.0);



        waitForStart();
        shooterMotor.setPower(.7);
        gateServoL.setPosition(0);
        gateServoR.setPosition(0);




        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(beginPose)
                                .strafeTo(new Vector2d(80, 0))
                                .stopAndAdd(new AutoShooterAction(hardwareMap))

                                .strafeTo(new Vector2d(73, 24))
                                .afterTime(0, new AutoIntakeAction(hardwareMap))
                                .strafeTo(new Vector2d(73, 34), slowVel, slowAccel)

                                .strafeTo(new Vector2d(80, 0))
                                .stopAndAdd(new AutoShooterAction(hardwareMap))

                                .strafeTo(new Vector2d(50, 24))
                                .afterTime(0, new AutoIntakeAction(hardwareMap))
                                .strafeTo(new Vector2d(50, 34), slowVel, slowAccel)


                                .strafeTo(new Vector2d(80, 0))
                                .stopAndAdd(new AutoShooterAction(hardwareMap))
                                .strafeTo(new Vector2d(62.5,35))

                                .build()

//                        drive.actionBuilder(beginPose)
//                                .stopAndAdd(new AutoShooterAction(hardwareMap))
//
//                                .strafeTo(new Vector2d(25, 24))
//                                .afterTime(0, new AutoIntakeAction(hardwareMap))
//                                .strafeTo(new Vector2d(25, 34), slowVel, slowAccel)
//
//                                .strafeTo(new Vector2d(0, 0))
//                                .stopAndAdd(new AutoShooterAction(hardwareMap))
//
//                                .strafeTo(new Vector2d(50, 24))
//                                .afterTime(0, new AutoIntakeAction(hardwareMap))
//                                .strafeTo(new Vector2d(50, 34), slowVel, slowAccel)
//
//
//                                .strafeTo(new Vector2d(0, 0))
//                                .stopAndAdd(new AutoShooterAction(hardwareMap))
//
//
//                                .strafeTo(new Vector2d(75, 24))
//                                .afterTime(0, new AutoIntakeAction(hardwareMap))
//                                .strafeTo(new Vector2d(75, 34), slowVel, slowAccel)
//
//
//                                .strafeTo(new Vector2d(0, 0))
//                                .stopAndAdd(new AutoShooterAction(hardwareMap))
//
//                                .strafeTo(new Vector2d(62.5,33))
//                                .build()
                )
        );
    }

    // =========================================================
    // Action 1: 自動吸球 (Intake Logic)
    // 功能：開始時升起閘門 (Open) -> 吸球 -> 結束
    // =========================================================
    public static class AutoIntakeAction implements Action {
        private final DcMotor intakeMotor;
        private final Servo diskServo;
        private final Servo gateServoL, gateServoR; // 閘門
        private final NormalizedColorSensor colorSensor1, colorSensor2;

        private boolean initialized = false;
        private long timer = 0;
        private int currentFillStep = 0;

        // === 參數設定 ===
        private static final double FILL_POS_STEP_1 = 0.0;
        private static final double FILL_POS_STEP_2 = 0.3529;
        private static final double FILL_POS_STEP_3 = 0.7137;
        private static final double INTAKE_POWER = 1.0;
        private static final int TIME_BALL_SETTLE = 200;
        private static final int TIME_DISK_MOVE = 300;

        // === 閘門開啟數值 (Updated) ===
        private static final double GATE_L_OPEN = 0.6667;
        private static final double GATE_R_OPEN = 0.6902;

        private enum State { IDLE, WAIT_SETTLE, ROTATING, FINISHED }
        private State state = State.IDLE;

        public AutoIntakeAction(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
            diskServo = hardwareMap.get(Servo.class, "servo2");

            // 初始化閘門
            gateServoL = hardwareMap.get(Servo.class, "servo4");
            gateServoR = hardwareMap.get(Servo.class, "servo5");

            // === 方向設定 (Updated) ===
            gateServoL.setDirection(Servo.Direction.REVERSE);
            gateServoR.setDirection(Servo.Direction.FORWARD);

            colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
            colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
            colorSensor1.setGain(25.0f);
            colorSensor2.setGain(25.0f);

            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // === 吸球開始：升起閘門 ===
                gateServoL.setPosition(GATE_L_OPEN);
                gateServoR.setPosition(GATE_R_OPEN);

                diskServo.setPosition(FILL_POS_STEP_1);
                intakeMotor.setPower(INTAKE_POWER);
                currentFillStep = 0;
                initialized = true;
            }

            switch (state) {
                case IDLE:
                    if (currentFillStep >= 3) {
                        state = State.FINISHED;
                        break;
                    }
                    if (isColorDetected()) {
                        timer = System.currentTimeMillis();
                        state = State.WAIT_SETTLE;
                    }
                    break;

                case WAIT_SETTLE:
                    if (System.currentTimeMillis() - timer > TIME_BALL_SETTLE) {
                        moveToNextPos();
                        timer = System.currentTimeMillis();
                        state = State.ROTATING;
                    }
                    break;

                case ROTATING:
                    if (System.currentTimeMillis() - timer > TIME_DISK_MOVE) {
                        state = State.IDLE;
                    }
                    break;

                case FINISHED:
                    intakeMotor.setPower(0);
                    return false; // Action 完成
            }

            packet.put("Intake State", state);
            packet.put("Balls", currentFillStep);
            return true; // Action 繼續
        }

        private void moveToNextPos() {
            if (currentFillStep == 0) { diskServo.setPosition(FILL_POS_STEP_2); currentFillStep = 1; }
            else if (currentFillStep == 1) { diskServo.setPosition(FILL_POS_STEP_3); currentFillStep = 2; }
            else if (currentFillStep == 2) { currentFillStep = 3; }
        }

        private boolean isColorDetected() {
            NormalizedRGBA c1 = colorSensor1.getNormalizedColors();
            NormalizedRGBA c2 = colorSensor2.getNormalizedColors();
            return (c1.alpha > 0.7 || c2.alpha > 0.7);
        }
    }

    // =========================================================
    // Action 2: 自動發射 (Shooter Logic)
    // 功能：開始時降下閘門 (Close) -> 發射 A/C/B -> 結束
    // =========================================================
    public static class AutoShooterAction implements Action {
        private final Servo diskServo, kickerServo;
        private final Servo gateServoL, gateServoR;

        private boolean initialized = false;
        private long timer = 0;

        // 狀態變數
        private boolean hasBallA = true, hasBallB = true, hasBallC = true;
        private String currentTarget = "";

        // === 參數設定 ===
        private static final double FIRE_POS_A = 0.8196;
        private static final double FIRE_POS_B = 0.0471;
        private static final double FIRE_POS_C = 0.4314;
        private static final double KICKER_REST = 0.0;
        private static final double KICKER_SHOOT = 0.8;

        // === 閘門數值 ===
        private static final double GATE_CLOSED = 0.0;
        private static final double GATE_L_OPEN = 0.6667;
        private static final double GATE_R_OPEN = 0.6902;

        // 【修改 1】 移除 SPIN_UP 狀態定義
        private enum State { DECIDE, AIMING, KICKING, RETRACTING, STOP }

        // 【修改 2】 初始狀態直接設為 DECIDE (雖然下面 initialized 會覆蓋，但這裡改了比較保險)
        private State state = State.DECIDE;

        public AutoShooterAction(HardwareMap hardwareMap) {
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
                // === 發射開始：降下閘門 (關閉) ===
                gateServoL.setPosition(GATE_CLOSED);
                gateServoR.setPosition(GATE_CLOSED);

                // 注意：這裡馬達已經開始轉了 (在 LinearOpMode 中設定的 power)
                // 如果需要在這裡才開啟馬達，記得加上 shooterMotor.setPower(0.75);

                timer = System.currentTimeMillis();

                // 【修改 3】 初始化完成後，直接進入 DECIDE 階段，跳過預熱
                state = State.DECIDE;

                initialized = true;
            }

            switch (state) {
                // 【修改 4】 case SPIN_UP 已被刪除

                case DECIDE:
                    // === 順序：C -> B -> A ===
                    if (hasBallC) {
                        diskServo.setPosition(FIRE_POS_C);
                        currentTarget = "C";
                        timer = System.currentTimeMillis();
                        state = State.AIMING;
                    }
                    else if (hasBallB) {
                        diskServo.setPosition(FIRE_POS_B);
                        currentTarget = "B";
                        timer = System.currentTimeMillis();
                        state = State.AIMING;
                    }
                    else if (hasBallA) {
                        diskServo.setPosition(FIRE_POS_A);
                        currentTarget = "A";
                        timer = System.currentTimeMillis();
                        state = State.AIMING;
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
                    // 發射結束：打開閘門
                    gateServoL.setPosition(GATE_L_OPEN);
                    gateServoR.setPosition(GATE_R_OPEN);

                    diskServo.setPosition(0.0);
                    return false; // Action 完成
            }

            packet.put("Shooter State", state);
            packet.put("Target", currentTarget);
            return true;
        }
    }
}