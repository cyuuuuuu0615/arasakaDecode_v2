package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop_v3")
public class teleop_v3 extends LinearOpMode {

    // === 硬件變量 ===
    NormalizedColorSensor colorSensor; // 安裝在 Position A (東面)
    Servo diskServo;

    // === 伺服機位置設定 ===
    // 0.00 -> Hole A 在 Sensor (初始)
    // 0.37 -> Hole B 在 Sensor
    // 0.73 -> Hole C 在 Sensor
    private static final double SERVO_POS_STEP_1 = 0.0;
    private static final double SERVO_POS_STEP_2 = 0.37;
    private static final double SERVO_POS_STEP_3 = 0.73;

    // === 顏色傳感器校準參數 ===
    private static final float SENSOR_GAIN = 25.0f;
    private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
    private static final float PURPLE_RATIO_LIMIT = 1.2f;

    // === 狀態追蹤 ===
    private int currentStep = 0;

    // 記錄每個洞的顏色
    private String colorHoleA = "EMPTY";
    private String colorHoleB = "EMPTY";
    private String colorHoleC = "EMPTY";

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    @Override
    public void runOpMode() {
        // 1. 初始化
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        diskServo = hardwareMap.get(Servo.class, "servo2");

        colorSensor.setGain(SENSOR_GAIN);

        // 初始歸零：Hole A 在 Position A (Sensor)
        diskServo.setPosition(SERVO_POS_STEP_1);
        currentStep = 0;

        telemetry.addData("Status", "Ready");
        telemetry.addData("Config", "Pos A=East(Sensor), Pos B=NW, Pos C=SW");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 獲取顏色
            DetectedColor detectedColor = getDetectedColor(colorSensor);

            // 只有當檢測到有效顏色 (非 UNKNOWN) 且還沒滿時才執行邏輯
            if (detectedColor != DetectedColor.UNKNOWN && currentStep < 3) {

                switch (currentStep) {
                    case 0: // t=1: 處理 Hole A
                        colorHoleA = detectedColor.toString();
                        telemetry.addData("Event", "Ball 1 Detected in Hole A: " + colorHoleA);
                        telemetry.update();

                        // 轉動: 讓 Hole B 轉到 Sensor
                        rotateDiskTo(SERVO_POS_STEP_2);
                        currentStep = 1;
                        break;

                    case 1: // t=2: 處理 Hole B (現在位於 Sensor)
                        colorHoleB = detectedColor.toString();
                        telemetry.addData("Event", "Ball 2 Detected in Hole B: " + colorHoleB);
                        telemetry.update();

                        // 轉動: 讓 Hole C 轉到 Sensor
                        rotateDiskTo(SERVO_POS_STEP_3);
                        currentStep = 2;
                        break;

                    case 2: // t=3: 處理 Hole C (現在位於 Sensor)
                        colorHoleC = detectedColor.toString();
                        telemetry.addData("Event", "Ball 3 Detected in Hole C: " + colorHoleC);
                        telemetry.update();

                        // 填滿結束
                        currentStep = 3;
                        break;
                }

                // 等待球完全落下，防止重複讀取
                sleep(1000);
            }

            // === Telemetry 顯示邏輯 (已修正方位) ===
            updateDetailedTelemetry();
        }
    }

    // 輔助方法：控制 Servo 並等待
    private void rotateDiskTo(double position) {
        diskServo.setPosition(position);
        sleep(500);
    }

    // 顯示詳細的位置和顏色信息
    private void updateDetailedTelemetry() {
        telemetry.addLine("=== SYSTEM STATUS ===");
        if (currentStep >= 3) {
            telemetry.addData("State", "FULL (All filled)");
        } else {
            String targetHole = (currentStep == 0) ? "A" : (currentStep == 1) ? "B" : "C";
            telemetry.addData("State", "Waiting for Hole " + targetHole);
        }

        telemetry.addLine("\n=== HOLE LOCATIONS ===");

        // 定義物理位置名稱
        String LOC_SENSOR = "Pos A (East/Sensor)";
        String LOC_NW = "Pos B (NW)";
        String LOC_SW = "Pos C (SW)";

        String posHoleA, posHoleB, posHoleC;

        if (currentStep == 0) {
            // 初始: A在Sensor, B在西北, C在西南
            posHoleA = LOC_SENSOR;
            posHoleB = LOC_NW;
            posHoleC = LOC_SW;
        } else if (currentStep == 1) {
            // 轉到 0.37: B來到Sensor
            // 順時針轉動: A去西南, C去西北
            posHoleA = LOC_SW;
            posHoleB = LOC_SENSOR;
            posHoleC = LOC_NW;
        } else {
            // 轉到 0.73: C來到Sensor
            // 再轉一次: A去西北, B去西南
            posHoleA = LOC_NW;
            posHoleB = LOC_SW;
            posHoleC = LOC_SENSOR;
        }

        // 格式化輸出
        telemetry.addData("Hole A", "[%s] is at %s", colorHoleA, posHoleA);
        telemetry.addData("Hole B", "[%s] is at %s", colorHoleB, posHoleB);
        telemetry.addData("Hole C", "[%s] is at %s", colorHoleC, posHoleC);

        telemetry.addLine("\n=== SENSOR DEBUG ===");
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addData("Detected", getDetectedColor(colorSensor));
        telemetry.addData("Alpha", "%.3f", colors.alpha);

        telemetry.update();
    }

    // 嚴格顏色檢測
    public DetectedColor getDetectedColor(NormalizedColorSensor sensor) {
        NormalizedRGBA color = sensor.getNormalizedColors();

        if (color.alpha < MIN_DETECT_BRIGHTNESS) {
            return DetectedColor.UNKNOWN;
        }

        if (color.blue > color.green && color.blue > color.red) {
            if (color.blue > (color.green * PURPLE_RATIO_LIMIT)) {
                return DetectedColor.PURPLE;
            }
        }

        if (color.green > color.red) {
            if (color.green >= color.blue || (color.green > color.blue * 0.85f)) {
                return DetectedColor.GREEN;
            }
        }

        return DetectedColor.UNKNOWN;
    }
}
//代碼邏輯說明
//狀態管理 (currentStep)：
//
//Step 0: 初始狀態。Color Sensor 對準 Hole A。檢測到球後，系統認為這是進入 Hole A 的球，記錄顏色，然後 Servo 轉動。
//
//Step 1: Servo 轉到了 0.37。現在 Hole C 被轉到了 Sensor 下方。檢測到球後，記錄為 Hole C 的顏色，Servo 再次轉動。
//
//Step 2: Servo 轉到了 0.73。現在 Hole B 被轉到了 Sensor 下方。檢測到球後，記錄為 Hole B 的顏色。
//
//Step 3: 鎖定狀態，不再轉動。
//
//Telemetry 動態顯示 (updateDetailedTelemetry)：
//
//這部分代碼非常重要。它不只是顯示顏色，還根據 currentStep 判斷每個洞現在轉到了哪裡。
//
//例如：當 currentStep == 1 時，顯示 "Hole A is at Position B (NW)"，這符合你描述的物理運動。
//
//防止重複檢測 (sleep)：
//
//我在 rotateDiskTo 中加了 sleep(500)，確保轉動時不會誤讀數據。
//
//在檢測成功後加了 sleep(1000)，這是為了讓球有時間完全跌入並穩定下來，避免同一個球觸發兩次 Sensor。你可以根據球跌落的速度調整這個時間。
//
//測試步驟
//初始位置：確保機器人啟動時，圓盤的 Hole A 對準東面（Color Sensor 下方）。
//
//投入第一顆球：放入 Sensor 下方。
//
//預期：Telemetry 顯示 "Ball 1 Detected in Hole A"，圓盤轉動。
//
//投入第二顆球：放入 Sensor 下方（此時應該是 Hole C 對準這裡）。
//
//預期：Telemetry 顯示 "Ball 2 Detected in Hole C"，圓盤轉動。
//
//投入第三顆球：放入 Sensor 下方（此時應該是 Hole B 對準這裡）。
//
//預期：Telemetry 顯示 "Ball 3 Detected in Hole B"，圓盤不轉動。
//
//檢查屏幕：Driver Station 應該顯示三個洞的顏色紀錄，以及它們最後停留的位置。