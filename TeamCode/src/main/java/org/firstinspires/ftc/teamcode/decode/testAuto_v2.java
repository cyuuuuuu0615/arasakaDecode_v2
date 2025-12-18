//package org.firstinspires.ftc.teamcode.decode;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class testAuto_v2 implements Action {
//
//    // === 硬件 ===
//    private final DcMotor intakeMotor;
//    private final Servo diskServo;
//    private final NormalizedColorSensor colorSensor1, colorSensor2;
//
//    // === 參數 (與 TeleOp 保持一致) ===
//    private static final double FILL_POS_STEP_1 = 0.0;     // Hole A
//    private static final double FILL_POS_STEP_2 = 0.3529;  // Hole B
//    private static final double FILL_POS_STEP_3 = 0.7137;  // Hole C
//    private static final double INTAKE_POWER = 1.0;
//
//    private static final int TIME_BALL_SETTLE = 800;
//    private static final int TIME_DISK_MOVE = 500;
//
//    // 傳感器參數
//    private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
//    private static final float PURPLE_RATIO_LIMIT = 1.2f;
//
//    // === 狀態機 ===
//    private enum State { INIT, IDLE, WAIT_SETTLE, ROTATING, FINISHED }
//    private State state = State.INIT;
//
//    // === 運行變量 ===
//    private long timer = 0;
//    private int currentFillStep = 0;
//    private boolean initialized = false;
//
//    // === 建構子 (Constructor) ===
//    public AutoIntakeAction(HardwareMap hardwareMap) {
//        // 在這裡獲取硬件
//        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
//        diskServo = hardwareMap.get(Servo.class, "servo2");
//        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
//        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
//
//        // 設定傳感器增益
//        colorSensor1.setGain(25.0f);
//        colorSensor2.setGain(25.0f);
//
//        // 設定光源
//        if (colorSensor1 instanceof com.qualcomm.robotcore.hardware.SwitchableLight) ((com.qualcomm.robotcore.hardware.SwitchableLight)colorSensor1).enableLight(true);
//        if (colorSensor2 instanceof com.qualcomm.robotcore.hardware.SwitchableLight) ((com.qualcomm.robotcore.hardware.SwitchableLight)colorSensor2).enableLight(true);
//    }
//
//    @Override
//    public boolean run(@NonNull TelemetryPacket packet) {
//
//        // 第一次運行時的初始化
//        if (!initialized) {
//            diskServo.setPosition(FILL_POS_STEP_1); // 歸零到 A
//            intakeMotor.setPower(INTAKE_POWER);     // 開啟 Intake
//            currentFillStep = 0;
//            state = State.IDLE;
//            initialized = true;
//        }
//
//        // === 狀態機邏輯 ===
//        switch (state) {
//            case IDLE:
//                // 檢查是否已滿 3 顆
//                if (currentFillStep >= 3) {
//                    state = State.FINISHED;
//                    break;
//                }
//
//                // 檢測顏色
//                if (getDualSensorColor() != DetectedColor.UNKNOWN) {
//                    timer = System.currentTimeMillis();
//                    state = State.WAIT_SETTLE;
//                }
//                break;
//
//            case WAIT_SETTLE:
//                // 等待球落穩 (800ms)
//                if (System.currentTimeMillis() - timer > TIME_BALL_SETTLE) {
//                    moveToNextFillPosition();
//                    timer = System.currentTimeMillis();
//                    state = State.ROTATING;
//                }
//                break;
//
//            case ROTATING:
//                // 等待 Servo 轉動 (500ms)
//                if (System.currentTimeMillis() - timer > TIME_DISK_MOVE) {
//                    // 轉動完成，回到 IDLE 繼續吸下一顆
//                    state = State.IDLE;
//                }
//                break;
//
//            case FINISHED:
//                // 動作完成
//                intakeMotor.setPower(0); // 關閉 Intake
//                return false; // 返回 false 表示 Action 結束
//        }
//
//        // === Telemetry 更新 ===
//        packet.put("Intake State", state);
//        packet.put("Current Step", currentFillStep);
//
//        // 返回 true 表示 Action 還在進行中
//        return true;
//    }
//
//    // === 輔助方法 (直接複製自 TeleOp) ===
//    private void moveToNextFillPosition() {
//        if (currentFillStep == 0) {
//            diskServo.setPosition(FILL_POS_STEP_2); // A -> B
//            currentFillStep = 1;
//        } else if (currentFillStep == 1) {
//            diskServo.setPosition(FILL_POS_STEP_3); // B -> C
//            currentFillStep = 2;
//        } else if (currentFillStep == 2) {
//            currentFillStep = 3; // 滿了
//        }
//    }
//
//    // 顏色檢測枚舉
//    private enum DetectedColor { PURPLE, GREEN, UNKNOWN }
//
//    private DetectedColor getDualSensorColor() {
//        DetectedColor c1 = getDetectedColor(colorSensor1);
//        DetectedColor c2 = getDetectedColor(colorSensor2);
//        if (c1 != DetectedColor.UNKNOWN) return c1;
//        if (c2 != DetectedColor.UNKNOWN) return c2;
//        return DetectedColor.UNKNOWN;
//    }
//
//    private DetectedColor getDetectedColor(NormalizedColorSensor sensor) {
//        NormalizedRGBA color = sensor.getNormalizedColors();
//        if (color.alpha < MIN_DETECT_BRIGHTNESS) return DetectedColor.UNKNOWN;
//
//        if (color.blue > color.green && color.blue > color.red) {
//            if (color.blue > (color.green * PURPLE_RATIO_LIMIT)) return DetectedColor.PURPLE;
//        }
//        if (color.green > color.red) {
//            if (color.green >= color.blue || (color.green > color.blue * 0.85f)) return DetectedColor.GREEN;
//        }
//        return DetectedColor.UNKNOWN;
//    }
//}