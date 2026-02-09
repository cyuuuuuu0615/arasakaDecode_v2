//package org.firstinspires.ftc.teamcode.decode;
//
//import androidx.annotation.NonNull;
//
//// RoadRunner imports
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//
//// Hardware imports
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//// Drive import
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Config
//@Autonomous(name = "testAuto_v2")
//public class testAuto_v2 extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // 1. 初始化底盤
//        Pose2d beginPose = new Pose2d(0, 0, 0);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
//
//        // 2. 獲取並初始化 Action
//        // 這裡我們直接使用下面定義的 class
//        Action intakeTask = new AutoIntakeAction(hardwareMap);
//        Action shooterTask = new AutoShooterAction(hardwareMap);
//
//        waitForStart();
//
//        // 3. 執行動作序列
//        Actions.runBlocking(
//                new SequentialAction(
//                        // 第一步：移動到吸球位置 (範例座標)
//
//                        drive.actionBuilder(beginPose)
//
//                                .afterTime(0, intakeTask)
//
//                                .strafeTo(new Vector2d(0, 33))
//
//                                .build()
////                        drive.actionBuilder(beginPose)
//////                                .stopAndAdd(shooterTask)
////                                .strafeTo(new Vector2d(0, 33))
////
////                                .stopAndAdd(intakeTask)
////
////                                .build()
//                        // 第四步：執行自動射擊 (A -> C -> B)
//
////000000000000000000000000000000
////                        drive.actionBuilder(new Pose2d(86.1, 0, 0))
////                                .strafeTo(new Vector2d(79, 0))
////                                .build(),
////                        drive.actionBuilder(new Pose2d(79, 0, 0))
////                        .strafeTo(new Vector2d(79, 33))
////                        .build(),
//                        //0000000000000000000000000
//                        // 第二步：執行自動吸球 (吸滿 3 顆才會結束)
////                        intakeTask,
//
//                        // 第三步：移動到射擊位置
////                        drive.actionBuilder(beginPose)
////                                .strafeTo(new Vector2d(86.1, 33))
////                                .build(),
////                        // 第四步：執行自動射擊 (A -> C -> B)
////                        shooterTask,
////
//
//
//
//                        // 第五步：停在原處或去停車
////                        drive.actionBuilder(new Pose2d(81.51, 56,0))
////                                .strafeTo(new Vector2d(0, 0))
////                                .build()
//                )
//        );
//
//    }
//
//    // =========================================================
//    // Action 1: 自動吸球 (Intake Logic)
//    // =========================================================
//    public static class AutoIntakeAction implements Action {
//        private final DcMotor intakeMotor;
//        private final Servo diskServo;
//        private final NormalizedColorSensor colorSensor1, colorSensor2;
//
//        private boolean initialized = false;
//        private long timer = 0;
//        private int currentFillStep = 0; // 0=Empty, 1=A滿, 2=B滿, 3=Full
//
//        // 參數
//        private static final double FILL_POS_STEP_1 = 0.0;     // Hole A
//        private static final double FILL_POS_STEP_2 = 0.3529;  // Hole B
//        private static final double FILL_POS_STEP_3 = 0.7137;  // Hole C
//        private static final double INTAKE_POWER = 1.0;
//        private static final int TIME_BALL_SETTLE = 800; // 等球落穩
//        private static final int TIME_DISK_MOVE = 500;   // 等Servo轉
//
//        // 狀態機
//        private enum State { IDLE, WAIT_SETTLE, ROTATING, FINISHED }
//        private State state = State.IDLE;
//
//        public AutoIntakeAction(HardwareMap hardwareMap) {
//            intakeMotor = hardwareMap.get(DcMotor.class, "motor4"); // 根據你的配置修改名稱
//            diskServo = hardwareMap.get(Servo.class, "servo2");
//            colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
//            colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
//
//            colorSensor1.setGain(25.0f);
//            colorSensor2.setGain(25.0f);
//            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            // 初始化
//            if (!initialized) {
//                diskServo.setPosition(FILL_POS_STEP_1);
//                intakeMotor.setPower(INTAKE_POWER);
//                currentFillStep = 0;
//                initialized = true;
//            }
//
//            switch (state) {
//                case IDLE:
//                    // 如果滿了 3 顆，結束
//                    if (currentFillStep >= 3) {
//                        state = State.FINISHED;
//                        break;
//                    }
//                    // 檢測顏色
//                    if (isColorDetected()) {
//                        timer = System.currentTimeMillis();
//                        state = State.WAIT_SETTLE;
//                    }
//                    break;
//
//                case WAIT_SETTLE:
//                    if (System.currentTimeMillis() - timer > TIME_BALL_SETTLE) {
//                        moveToNextPos();
//                        timer = System.currentTimeMillis();
//                        state = State.ROTATING;
//                    }
//                    break;
//
//                case ROTATING:
//                    if (System.currentTimeMillis() - timer > TIME_DISK_MOVE) {
//                        state = State.IDLE; // 轉完後回到 IDLE 繼續吸下一顆
//                    }
//                    break;
//
//                case FINISHED:
//                    intakeMotor.setPower(0);
//                    return false; // 返回 false 代表此 Action 結束
//            }
//
//            // 顯示資訊
//            packet.put("Intake State", state);
//            packet.put("Balls Collected", currentFillStep);
//            return true; // 返回 true 代表 Action 還沒做完，繼續執行
//        }
//
//        private void moveToNextPos() {
//            if (currentFillStep == 0) { diskServo.setPosition(FILL_POS_STEP_2); currentFillStep = 1; }
//            else if (currentFillStep == 1) { diskServo.setPosition(FILL_POS_STEP_3); currentFillStep = 2; }
//            else if (currentFillStep == 2) { currentFillStep = 3; }
//        }
//
//        private boolean isColorDetected() {
//            NormalizedRGBA c1 = colorSensor1.getNormalizedColors();
//            NormalizedRGBA c2 = colorSensor2.getNormalizedColors();
//            // 簡單亮度檢測，大於 0.7 視為有球
//            return (c1.alpha > 0.7 || c2.alpha > 0.7);
//        }
//    }
//
//    // =========================================================
//    // Action 2: 自動發射 (Shooter Logic - A -> C -> B)
//    // =========================================================
//    public static class AutoShooterAction implements Action {
//        private final DcMotor shooterMotor;
//        private final Servo diskServo, kickerServo;
//        private boolean initialized = false;
//        private long timer = 0;
//
//        // 假設 Intake 結束時我們有 A, B, C 三顆球
//        private boolean hasBallA = true, hasBallB = true, hasBallC = true;
//        private String currentTarget = "";
//
//        // 參數
//        private static final double FIRE_POS_A = 0.8196;
//        private static final double FIRE_POS_B = 0.0471;
//        private static final double FIRE_POS_C = 0.4314;
//        private static final double KICKER_REST = 0.0;
//        private static final double KICKER_SHOOT = 0.8;
//
//        // 狀態機
//        private enum State { SPIN_UP, DECIDE, AIMING, KICKING, RETRACTING, STOP }
//        private State state = State.SPIN_UP;
//
//        public AutoShooterAction(HardwareMap hardwareMap) {
//            shooterMotor = hardwareMap.get(DcMotor.class, "motor5"); // 根據你的配置修改
//            diskServo = hardwareMap.get(Servo.class, "servo2");
//            kickerServo = hardwareMap.get(Servo.class, "servo1");
//
//            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            kickerServo.setPosition(KICKER_REST);
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
//                shooterMotor.setPower(0.4); // 啟動摩擦輪
//                timer = System.currentTimeMillis();
//                state = State.SPIN_UP;
//                initialized = true;
//            }
//
//            switch (state) {
//                case SPIN_UP:
//                    // 等待馬達加速 1秒
//                    if (System.currentTimeMillis() - timer > 1000) {
//                        state = State.DECIDE;
//                    }
//                    break;
//
//                case DECIDE:
//                    // 優先順序 A -> C -> B
//                    if (hasBallA) {
//                        diskServo.setPosition(FIRE_POS_A);
//                        currentTarget = "A";
//                        timer = System.currentTimeMillis();
//                        state = State.AIMING;
//                    } else if (hasBallC) {
//                        diskServo.setPosition(FIRE_POS_C);
//                        currentTarget = "C";
//                        timer = System.currentTimeMillis();
//                        state = State.AIMING;
//                    } else if (hasBallB) {
//                        diskServo.setPosition(FIRE_POS_B);
//                        currentTarget = "B";
//                        timer = System.currentTimeMillis();
//                        state = State.AIMING;
//                    } else {
//                        // 球都射完了
//                        state = State.STOP;
//                    }
//                    break;
//
//                case AIMING:
//                    // 等待 Servo 轉到位 (500ms)
//                    if (System.currentTimeMillis() - timer > 500) {
//                        kickerServo.setPosition(KICKER_SHOOT); // 踢球
//                        timer = System.currentTimeMillis();
//                        state = State.KICKING;
//                    }
//                    break;
//
//                case KICKING:
//                    // 等待踢出 (300ms)
//                    if (System.currentTimeMillis() - timer > 300) {
//                        kickerServo.setPosition(KICKER_REST); // 收回
//                        // 標記該球已發射
//                        if (currentTarget.equals("A")) hasBallA = false;
//                        if (currentTarget.equals("B")) hasBallB = false;
//                        if (currentTarget.equals("C")) hasBallC = false;
//
//                        timer = System.currentTimeMillis();
//                        state = State.RETRACTING;
//                    }
//                    break;
//
//                case RETRACTING:
//                    // 等待收回 (250ms)
//                    if (System.currentTimeMillis() - timer > 250) {
//                        state = State.DECIDE; // 回去決定下一顆
//                    }
//                    break;
//
//                case STOP:
//                    shooterMotor.setPower(0); // 關閉馬達
//                    diskServo.setPosition(0.0); // 歸位
//                    return false; // Action 結束
//            }
//
//            packet.put("Shooter State", state);
//            packet.put("Target", currentTarget);
//            return true;
//        }
//    }
//}