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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
@Autonomous(name = "testAuto_v4")
public class testAuto_v4 extends LinearOpMode {

    // === 定義顏色與全域變數 ===
    public enum BallColor {
        PURPLE, GREEN, NONE
    }

    // AprilTag 決定的發射順序 (例如: [GREEN, PURPLE, PURPLE])
    public static List<BallColor> targetSequence = new ArrayList<>();

    // 彈倉目前的狀態：Index 0, 1, 2 對應吸進來的第一、二、三顆球
    public static BallColor[] actualBallSlots = {BallColor.NONE, BallColor.NONE, BallColor.NONE};

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- 硬體初始化 ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "motor5");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo gateServoL = hardwareMap.get(Servo.class, "servo4");
        Servo gateServoR = hardwareMap.get(Servo.class, "servo5");
        gateServoL.setDirection(Servo.Direction.REVERSE);
        gateServoR.setDirection(Servo.Direction.FORWARD);

        // --- 速度限制 ---
        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10.0),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint slowAccel = new ProfileAccelConstraint(-10.0, 10.0);

        // 預設發射順序 (防止沒掃到)
        targetSequence.clear();
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);
        targetSequence.add(BallColor.PURPLE);

        telemetry.addLine("v4 Ready. Optimization: Shortest Servo Path.");
        telemetry.update();

        // === Init Loop: 掃描 AprilTag ===
        while (opModeInInit()) {
            // 這裡模擬讀取 Tag ID 邏輯，請替換為實際 Limelight 獲取 ID 的程式碼
            // 假設我們獲取到了一個 id 變數
            int id = -1;
            // LLResult result = limelight.getLatestResult();
            // if (result != null) { ... get id ... }

            // 模擬邏輯：(請確保您的 Limelight pipeline 設定正確)
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
            }
            telemetry.update();
        }

        waitForStart();
        limelight.stop();

        // 初始設定
        shooterMotor.setPower(0.75);
        gateServoL.setPosition(0);
        gateServoR.setPosition(0);

        // 假設初始彈倉為空
        Arrays.fill(actualBallSlots, BallColor.NONE);

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(beginPose)
                                .strafeTo(new Vector2d(80, 0))
                                .stopAndAdd(new AutoShooterAction(hardwareMap))

                                .strafeTo(new Vector2d(75, 24))
                                .afterTime(0, new AutoIntakeAction(hardwareMap))
                                .strafeTo(new Vector2d(75, 34), slowVel, slowAccel)

                                .strafeTo(new Vector2d(80, 0))
                                .stopAndAdd(new AutoShooterAction(hardwareMap))

                                .strafeTo(new Vector2d(50, 24))
                                .afterTime(0, new AutoIntakeAction(hardwareMap))
                                .strafeTo(new Vector2d(50, 34), slowVel, slowAccel)

                                .strafeTo(new Vector2d(80, 0))
                                .stopAndAdd(new AutoShooterAction(hardwareMap))

                                .build()
                )
        );
    }

    // =========================================================
    // Action 1: AutoIntakeAction (負責辨識並填入顏色)
    // =========================================================
    public static class AutoIntakeAction implements Action {
        private final DcMotor intakeMotor;
        private final Servo diskServo;
        private final Servo gateServoL, gateServoR;
        private final NormalizedColorSensor colorSensor1, colorSensor2;

        private boolean initialized = false;
        private long timer = 0;
        private int currentFillStep = 0;

        private static final double FILL_POS_STEP_1 = 0.0;
        private static final double FILL_POS_STEP_2 = 0.3529;
        private static final double FILL_POS_STEP_3 = 0.7137;
        private static final double INTAKE_POWER = 1.0;
        private static final int TIME_BALL_SETTLE = 200;
        private static final int TIME_DISK_MOVE = 300;
        private static final double GATE_L_OPEN = 0.6667;
        private static final double GATE_R_OPEN = 0.6902;

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
                // 重置全域彈倉狀態
                Arrays.fill(testAuto_v4.actualBallSlots, BallColor.NONE);
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
                        identifyAndStoreColor(currentFillStep);
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
                    return false;
            }
            packet.put("Intake Balls", currentFillStep);
            return true;
        }

        private void identifyAndStoreColor(int slotIndex) {
            if (slotIndex > 2) return;
            NormalizedRGBA c1 = colorSensor1.getNormalizedColors();
            // 簡單顏色判斷 (需依現場校正)
            boolean isGreen = (c1.green > (c1.red + c1.blue) * 0.8);
            testAuto_v4.actualBallSlots[slotIndex] = isGreen ? BallColor.GREEN : BallColor.PURPLE;
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
    // Action 2: AutoShooterAction (路徑優化版)
    // =========================================================
    public static class AutoShooterAction implements Action {
        private final Servo diskServo, kickerServo;
        private final Servo gateServoL, gateServoR;

        private boolean initialized = false;
        private long timer = 0;

        private int currentTargetIndex = 0;
        private String currentAimTarget = "";

        // 用來記錄 Servo 最後的位置，計算最短路徑
        // 初始值設為 0.0 (或上一次 Intake 結束的位置)
        private double lastServoPos = 0.0;

        // === 發射位置 ===
        private static final double FIRE_POS_A = 0.8196;
        private static final double FIRE_POS_B = 0.0471;
        private static final double FIRE_POS_C = 0.4314;

        // 對應關係 (假設)：
        // Intake Index 2 -> Position A
        // Intake Index 1 -> Position B
        // Intake Index 0 -> Position C

        private static final double KICKER_REST = 0.0;
        private static final double KICKER_SHOOT = 0.8;
        private static final double GATE_CLOSED = 0.0;
        private static final double GATE_L_OPEN = 0.6667;
        private static final double GATE_R_OPEN = 0.6902;

        private enum State { DECIDE, AIMING, KICKING, RETRACTING, STOP }
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
                gateServoL.setPosition(GATE_CLOSED);
                gateServoR.setPosition(GATE_CLOSED);
                timer = System.currentTimeMillis();
                currentTargetIndex = 0;
                // 讀取當前 Servo 實際位置作為起點
                lastServoPos = diskServo.getPosition();
                state = State.DECIDE;
                initialized = true;
            }

            switch (state) {
                case DECIDE:
                    // 1. 檢查是否射完
                    if (currentTargetIndex >= 3) {
                        // 清空剩餘球 (如果有的話)
                        String leftover = findClosestAvailableSlot();
                        if (leftover != null) {
                            setupShot(leftover);
                            state = State.AIMING;
                        } else {
                            state = State.STOP;
                        }
                        break;
                    }

                    // 2. 獲取需要的顏色
                    BallColor neededColor = BallColor.PURPLE;
                    if (currentTargetIndex < testAuto_v4.targetSequence.size()) {
                        neededColor = testAuto_v4.targetSequence.get(currentTargetIndex);
                    }

                    // 3. 【關鍵優化】尋找擁有該顏色的「最近」彈倉
                    String targetSlot = findBestSlotForColor(neededColor);

                    if (targetSlot != null) {
                        setupShot(targetSlot);
                        state = State.AIMING;
                    } else {
                        // 沒球可射，跳過
                        currentTargetIndex++;
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
                        currentTargetIndex++;
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
            packet.put("State", state);
            packet.put("Target", currentAimTarget);
            return true;
        }

        private void setupShot(String slot) {
            currentAimTarget = slot;
            double targetPos = 0;

            if (slot.equals("A")) targetPos = FIRE_POS_A;
            else if (slot.equals("B")) targetPos = FIRE_POS_B;
            else if (slot.equals("C")) targetPos = FIRE_POS_C;

            diskServo.setPosition(targetPos);
            lastServoPos = targetPos; // 更新當前位置紀錄
            timer = System.currentTimeMillis();
        }

        // === 優化核心：找出距離當前位置最近的、且符合顏色的彈倉 ===
        private String findBestSlotForColor(BallColor color) {
            String bestSlot = null;
            double minDiff = 1000.0; // 設一個大數

            // 檢查 Slot 2 (A: 0.8196)
            if (testAuto_v4.actualBallSlots[2] == color) {
                double diff = Math.abs(lastServoPos - FIRE_POS_A);
                if (diff < minDiff) {
                    minDiff = diff;
                    bestSlot = "A";
                }
            }

            // 檢查 Slot 1 (B: 0.0471)
            if (testAuto_v4.actualBallSlots[1] == color) {
                double diff = Math.abs(lastServoPos - FIRE_POS_B);
                if (diff < minDiff) {
                    minDiff = diff;
                    bestSlot = "B";
                }
            }

            // 檢查 Slot 0 (C: 0.4314)
            if (testAuto_v4.actualBallSlots[0] == color) {
                double diff = Math.abs(lastServoPos - FIRE_POS_C);
                if (diff < minDiff) {
                    minDiff = diff;
                    bestSlot = "C";
                }
            }

            return bestSlot;
        }

        // 用於最後清空：找出最近的任何一顆球
        private String findClosestAvailableSlot() {
            String bestSlot = null;
            double minDiff = 1000.0;

            if (testAuto_v4.actualBallSlots[2] != BallColor.NONE) {
                double diff = Math.abs(lastServoPos - FIRE_POS_A);
                if (diff < minDiff) { minDiff = diff; bestSlot = "A"; }
            }
            if (testAuto_v4.actualBallSlots[1] != BallColor.NONE) {
                double diff = Math.abs(lastServoPos - FIRE_POS_B);
                if (diff < minDiff) { minDiff = diff; bestSlot = "B"; }
            }
            if (testAuto_v4.actualBallSlots[0] != BallColor.NONE) {
                double diff = Math.abs(lastServoPos - FIRE_POS_C);
                if (diff < minDiff) { minDiff = diff; bestSlot = "C"; }
            }
            return bestSlot;
        }

        private void removeBallFromSlot(String slot) {
            if (slot.equals("A")) testAuto_v4.actualBallSlots[2] = BallColor.NONE;
            else if (slot.equals("B")) testAuto_v4.actualBallSlots[1] = BallColor.NONE;
            else if (slot.equals("C")) testAuto_v4.actualBallSlots[0] = BallColor.NONE;
        }
    }
}