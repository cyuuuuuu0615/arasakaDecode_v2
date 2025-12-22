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
@Autonomous(name = "testAuto_v5")
public class testAuto_v5 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // 1. 初始化底盤
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // 2. 初始化射擊馬達 (保持運轉)
        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "motor5");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo gateServoL = hardwareMap.get(Servo.class, "servo4");
        Servo gateServoR = hardwareMap.get(Servo.class, "servo5");
        gateServoL.setDirection(Servo.Direction.REVERSE);
        gateServoR.setDirection(Servo.Direction.FORWARD);

        // 3. 建立分開的 Action 物件
        // 這樣我們才能在移動中呼叫 shooter.prepareAction()
        AutoShooterController shooter = new AutoShooterController(hardwareMap);
        AutoIntakeController intake = new AutoIntakeController(hardwareMap);

        // 速度限制
        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10.0),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint slowAccel = new ProfileAccelConstraint(-10.0, 10.0);

        waitForStart();

        // 初始狀態
        shooterMotor.setPower(0.75);
        gateServoL.setPosition(0);
        gateServoR.setPosition(0);

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(beginPose)
                                // === 第一發 ===
                                // .afterTime(0, ...) 表示：路徑一開始，馬上執行 Servo 預備 (Prepare)
                                // 車子在跑的時候，Servo 就在找最短路徑轉動了
                                .afterTime(0, shooter.prepareAction())
                                .strafeTo(new Vector2d(80, 0))

                                // 到達目的地後，Servo 早就到了，直接開火 (Fire)
                                .stopAndAdd(shooter.fireAction())

                                // === 吸球 (邊走邊吸) ===
                                .strafeTo(new Vector2d(75, 24))
                                // 移動開始時，放下吸球機構並開始旋轉
                                .afterTime(0, intake.startIntakeAction())
                                .strafeTo(new Vector2d(75, 34), slowVel, slowAccel)

                                // === 第二發 ===
                                // 移動去射擊點的同時：關閉吸球、預備 Servo
                                .afterTime(0, intake.stopIntakeAction())
                                // 這裡 delay 0.5秒是為了讓球有時間吸入定位後再轉 Servo，避免卡住
                                .afterTime(0.5, shooter.prepareAction())
                                .strafeTo(new Vector2d(80, 0))

                                .stopAndAdd(shooter.fireAction())

                                // === 第三發 ===
                                .strafeTo(new Vector2d(50, 24))
                                .afterTime(0, intake.startIntakeAction())
                                .strafeTo(new Vector2d(50, 34), slowVel, slowAccel)

                                .afterTime(0, intake.stopIntakeAction())
                                .afterTime(0.5, shooter.prepareAction())
                                .strafeTo(new Vector2d(80, 0))

                                .stopAndAdd(shooter.fireAction())

                                // 結束
                                .strafeTo(new Vector2d(62.5,33))
                                .build()
                )
        );
    }

    // =========================================================
    // Controller 1: 射擊控制 (拆分為 Prepare 和 Fire)
    // =========================================================
    public static class AutoShooterController {
        private final Servo diskServo, kickerServo;
        private final Servo gateServoL, gateServoR;

        // 狀態追蹤
        private boolean hasBallA = true, hasBallB = true, hasBallC = true;
        private String currentTargetSlot = ""; // "A", "B", or "C"
        private double currentServoPos = 0.0;  // 追蹤目前 Servo 在哪，用於算最短距離

        // 參數
        private static final double FIRE_POS_A = 0.8196;
        private static final double FIRE_POS_B = 0.0471;
        private static final double FIRE_POS_C = 0.4314;
        private static final double KICKER_REST = 0.0;
        private static final double KICKER_SHOOT = 0.8;
        private static final double GATE_CLOSED = 0.0;
        private static final double GATE_L_OPEN = 0.6667;
        private static final double GATE_R_OPEN = 0.6902;

        public AutoShooterController(HardwareMap hardwareMap) {
            diskServo = hardwareMap.get(Servo.class, "servo2");
            kickerServo = hardwareMap.get(Servo.class, "servo1");
            gateServoL = hardwareMap.get(Servo.class, "servo4");
            gateServoR = hardwareMap.get(Servo.class, "servo5");

            gateServoL.setDirection(Servo.Direction.REVERSE);
            gateServoR.setDirection(Servo.Direction.FORWARD);

            // 初始復位
            kickerServo.setPosition(KICKER_REST);
            currentServoPos = 0.0;
        }

        // --- Action A: 預備動作 (在移動中執行) ---
        // 功能：計算最短路徑 -> 轉動 Servo -> 關閉閘門
        public Action prepareAction() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        // 1. 關閘門
                        gateServoL.setPosition(GATE_CLOSED);
                        gateServoR.setPosition(GATE_CLOSED);

                        // 2. 計算最短距離的目標
                        String bestSlot = findBestSlot();

                        // 3. 轉動 Servo
                        if (bestSlot != null) {
                            double targetPos = getPosFromSlot(bestSlot);
                            diskServo.setPosition(targetPos);

                            // 更新狀態
                            currentServoPos = targetPos;
                            currentTargetSlot = bestSlot;
                        }

                        initialized = true;
                    }
                    // 回傳 false 代表動作瞬間完成 (不阻擋車子移動)
                    // RoadRunner 會繼續執行 strafeTo，同時 Servo 正在轉
                    return false;
                }
            };
        }

        // --- Action B: 發射動作 (到達後執行) ---
        // 功能：AIMING(300ms) -> KICKING(300ms) -> RETRACTING(100ms)
        public Action fireAction() {
            return new Action() {
                private boolean initialized = false;
                private long timer;
                private State state = State.AIMING;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        timer = System.currentTimeMillis();
                        state = State.AIMING;
                        initialized = true;
                    }

                    switch (state) {
                        case AIMING:
                            // 這裡是你要求的 300ms 等待 (確保 Servo 停穩)
                            if (System.currentTimeMillis() - timer > 300) {
                                kickerServo.setPosition(KICKER_SHOOT);
                                timer = System.currentTimeMillis();
                                state = State.KICKING;
                            }
                            break;

                        case KICKING:
                            // 這裡是你要求的 300ms (踢球動作)
                            if (System.currentTimeMillis() - timer > 300) {
                                kickerServo.setPosition(KICKER_REST);
                                updateInventory(currentTargetSlot); // 扣除球
                                timer = System.currentTimeMillis();
                                state = State.RETRACTING;
                            }
                            break;

                        case RETRACTING:
                            // 這裡是你要求的縮短版 100ms
                            if (System.currentTimeMillis() - timer > 100) {
                                // 打開閘門準備吸球 (如果需要的話，或者保持關閉)
                                gateServoL.setPosition(GATE_L_OPEN);
                                gateServoR.setPosition(GATE_R_OPEN);
                                diskServo.setPosition(0.0); // 歸零或保持
                                currentServoPos = 0.0;
                                return false; // 完成
                            }
                            break;
                    }
                    packet.put("Shooter State", state);
                    return true;
                }
            };
        }

        // 輔助函式：貪婪演算法 (找最近的球)
        private String findBestSlot() {
            String best = null;
            double minDiff = 100.0; // 設一個大數

            // 檢查 A
            if (hasBallA) {
                double diff = Math.abs(currentServoPos - FIRE_POS_A);
                if (diff < minDiff) { minDiff = diff; best = "A"; }
            }
            // 檢查 B
            if (hasBallB) {
                double diff = Math.abs(currentServoPos - FIRE_POS_B);
                if (diff < minDiff) { minDiff = diff; best = "B"; }
            }
            // 檢查 C
            if (hasBallC) {
                double diff = Math.abs(currentServoPos - FIRE_POS_C);
                if (diff < minDiff) { minDiff = diff; best = "C"; }
            }
            return best;
        }

        private double getPosFromSlot(String slot) {
            if (slot.equals("A")) return FIRE_POS_A;
            if (slot.equals("B")) return FIRE_POS_B;
            if (slot.equals("C")) return FIRE_POS_C;
            return 0.0;
        }

        private void updateInventory(String slot) {
            if (slot.equals("A")) hasBallA = false;
            if (slot.equals("B")) hasBallB = false;
            if (slot.equals("C")) hasBallC = false;
        }

        private enum State { AIMING, KICKING, RETRACTING }
    }

    // =========================================================
    // Controller 2: 吸球控制 (拆分為 Start 和 Stop)
    // =========================================================
    public static class AutoIntakeController {
        private final DcMotor intakeMotor;
        private final Servo diskServo, gateServoL, gateServoR;
        private final NormalizedColorSensor colorSensor1, colorSensor2;

        private static final double INTAKE_POWER = 1.0;
        // 吸球時的簡單轉動邏輯，或者只負責開馬達
        // 為了簡化 Parallel Action，這裡主要負責開關馬達與放下閘門

        public AutoIntakeController(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
            diskServo = hardwareMap.get(Servo.class, "servo2");
            gateServoL = hardwareMap.get(Servo.class, "servo4");
            gateServoR = hardwareMap.get(Servo.class, "servo5");

            gateServoL.setDirection(Servo.Direction.REVERSE);
            gateServoR.setDirection(Servo.Direction.FORWARD);

            colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
            colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Action: 開始吸球 (放下閘門、轉動馬達)
        public Action startIntakeAction() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        gateServoL.setPosition(0.6667); // Open
                        gateServoR.setPosition(0.6902); // Open
                        diskServo.setPosition(0.0);     // Reset Position
                        intakeMotor.setPower(INTAKE_POWER);
                        initialized = true;
                    }
                    // 這裡可以加入簡單的 "如果滿了就停" 的邏輯
                    // 但為了不阻擋移動，我們先回傳 false，讓它在背景執行 (馬達已經開啟)
                    return false;
                }
            };
        }

        // Action: 停止吸球
        public Action stopIntakeAction() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    intakeMotor.setPower(0);
                    return false;
                }
            };
        }
    }
}