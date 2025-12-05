package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop_v4")
public class teleop_v4 extends LinearOpMode {

    // === 硬件變量 ===
    NormalizedColorSensor colorSensor;
    Servo kickerServo; // servo1: 長棍 (踢球用)
    Servo diskServo;   // servo2: 圓盤 (轉動用)

    // === 裝球位置 (Filling Positions) ===
    // 這是球進入的位置 (Sensor 位置)
    private static final double FILL_POS_STEP_1 = 0.0;  // Hole A 在 Sensor
    private static final double FILL_POS_STEP_2 = 0.37; // Hole B 在 Sensor
    private static final double FILL_POS_STEP_3 = 0.73; // Hole C 在 Sensor

    // === 發射位置 (Firing Positions) ===
    // 這是球對準長棍的位置
    private static final double FIRE_POS_HOLE_B = 0.0706;
    private static final double FIRE_POS_HOLE_C = 0.481;
    private static final double FIRE_POS_HOLE_A = 0.8039;

    // === 長棍 (Kicker) 參數 ===
    private static final double KICKER_REST = 0.0;
    private static final double KICKER_EXTEND = 0.8;
    private static final int KICKER_WAIT_MS = 300;

    // === 傳感器參數 ===
    private static final float SENSOR_GAIN = 25.0f;
    private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
    private static final float PURPLE_RATIO_LIMIT = 1.2f;

    // === 狀態追蹤 ===
    private int currentFillStep = 0; // 0=Fill A, 1=Fill B, 2=Fill C, 3=Full

    // 追蹤每個洞是否有球 (用於判斷是否需要發射)
    private boolean hasBallA = false;
    private boolean hasBallB = false;
    private boolean hasBallC = false;

    // 記錄顏色 (僅用於顯示)
    private String colorHoleA = "EMPTY";
    private String colorHoleB = "EMPTY";
    private String colorHoleC = "EMPTY";

    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }

    @Override
    public void runOpMode() {
        // 1. 初始化硬件
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        kickerServo = hardwareMap.get(Servo.class, "servo1"); // 長棍
        diskServo = hardwareMap.get(Servo.class, "servo2");   // 圓盤





        colorSensor.setGain(SENSOR_GAIN);

        // 初始歸位
        kickerServo.setPosition(KICKER_REST);
        diskServo.setPosition(FILL_POS_STEP_1);

        telemetry.addData("Status", "System Ready");
        telemetry.addData("Controls", "Auto-Fill active. Press LEFT BUMPER to Fire.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {





            // === 觸發發射模式 ===
            if (gamepad1.left_bumper) {
                fireAllBalls();
            }

            // === 自動裝球邏輯 (只有在沒滿的情況下執行) ===
            if (currentFillStep < 3) {
                handleFillingLogic();
            }

            // === 顯示狀態 ===
            updateTelemetry();
        }
    }

    // === 核心邏輯：裝球 (Filling) ===
    private void handleFillingLogic() {
        DetectedColor detectedColor = getDetectedColor(colorSensor);

        if (detectedColor != DetectedColor.UNKNOWN) {
            switch (currentFillStep) {
                case 0: // 裝 Hole A
                    colorHoleA = detectedColor.toString();
                    hasBallA = true;
                    telemetry.addData("Event", "Got Ball in Hole A");
                    telemetry.update();

                    rotateDiskTo(FILL_POS_STEP_2); // 轉去裝 B
                    currentFillStep = 1;
                    break;

                case 1: // 裝 Hole B
                    colorHoleB = detectedColor.toString();
                    hasBallB = true;
                    telemetry.addData("Event", "Got Ball in Hole B");
                    telemetry.update();

                    rotateDiskTo(FILL_POS_STEP_3); // 轉去裝 C
                    currentFillStep = 2;
                    break;

                case 2: // 裝 Hole C
                    colorHoleC = detectedColor.toString();
                    hasBallC = true;
                    telemetry.addData("Event", "Got Ball in Hole C");
                    telemetry.update();

                    // 滿了，不用轉，或者可以轉到特定位置等待
                    currentFillStep = 3;
                    break;
            }
            // 防止重複檢測同一顆球
            sleep(1000);
        }
    }

    // === 核心邏輯：發射 (Firing) ===
    private void fireAllBalls() {
        telemetry.addData("Mode", "FIRING SEQUENCE STARTED");
        telemetry.update();

        // 當還有任何一顆球在洞裡時，持續執行
        while (opModeIsActive() && (hasBallA || hasBallB || hasBallC)) {

            double currentPos = diskServo.getPosition();

            // 計算距離 (如果洞是空的，距離設為無限大 999，這樣就不會選中它)
            double distA = hasBallA ? Math.abs(currentPos - FIRE_POS_HOLE_A) : 999.0;
            double distB = hasBallB ? Math.abs(currentPos - FIRE_POS_HOLE_B) : 999.0;
            double distC = hasBallC ? Math.abs(currentPos - FIRE_POS_HOLE_C) : 999.0;

            // 選擇最短路徑
            if (distA <= distB && distA <= distC) {
                // 去射 A
                performShot(FIRE_POS_HOLE_A, "A");
                hasBallA = false;
                colorHoleA = "EMPTY";
            } else if (distB <= distA && distB <= distC) {
                // 去射 B
                performShot(FIRE_POS_HOLE_B, "B");
                hasBallB = false;
                colorHoleB = "EMPTY";
            } else {
                // 去射 C
                performShot(FIRE_POS_HOLE_C, "C");
                hasBallC = false;
                colorHoleC = "EMPTY";
            }
        }

        // 發射完畢，重置系統以接收新球
        telemetry.addData("Mode", "FIRING DONE - RESETTING");
        telemetry.update();

        // 回到初始裝球位置 (Hole A at Sensor)
        rotateDiskTo(FILL_POS_STEP_1);
        currentFillStep = 0;

        sleep(500); // 稍作停頓
    }

    // 執行單次射擊動作
    private void performShot(double targetDiskPos, String holeName) {
        telemetry.addData("Action", "Moving to Hole " + holeName + " (" + targetDiskPos + ")");
        telemetry.update();

        // 1. 轉動圓盤到位
        diskServo.setPosition(targetDiskPos);
        sleep(500); // 等待圓盤轉到位

        // 2. 踢球 (Kicker 動作)
        kickerServo.setPosition(KICKER_EXTEND); // 伸出 (0.8)
        sleep(KICKER_WAIT_MS);                  // 停頓 300ms
        kickerServo.setPosition(KICKER_REST);   // 收回 (0.0)
        sleep(200);                             // 等待收回
    }

    // === 輔助方法 ===
    private void rotateDiskTo(double position) {
        diskServo.setPosition(position);
        sleep(500);
    }

    private void updateTelemetry() {
        telemetry.addLine("=== SYSTEM STATUS ===");
        if (currentFillStep >= 3) {
            telemetry.addData("State", "FULL - Ready to Fire");
        } else {
            telemetry.addData("State", "Filling... (" + currentFillStep + "/3)");
        }

        telemetry.addLine("\n=== BALL STORAGE ===");
        telemetry.addData("Hole A", "[%s] %s", colorHoleA, hasBallA ? "(Has Ball)" : "");
        telemetry.addData("Hole B", "[%s] %s", colorHoleB, hasBallB ? "(Has Ball)" : "");
        telemetry.addData("Hole C", "[%s] %s", colorHoleC, hasBallC ? "(Has Ball)" : "");

        telemetry.update();
    }

    // 顏色檢測 (保持不變)
    public DetectedColor getDetectedColor(NormalizedColorSensor sensor) {
        NormalizedRGBA color = sensor.getNormalizedColors();
        if (color.alpha < MIN_DETECT_BRIGHTNESS) return DetectedColor.UNKNOWN;

        if (color.blue > color.green && color.blue > color.red) {
            if (color.blue > (color.green * PURPLE_RATIO_LIMIT)) return DetectedColor.PURPLE;
        }
        if (color.green > color.red) {
            if (color.green >= color.blue || (color.green > color.blue * 0.85f)) return DetectedColor.GREEN;
        }
        return DetectedColor.UNKNOWN;
    }
}