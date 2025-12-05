package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "ColorSensorTest_AI_v2")
public class ColorSensorTest_AI_v2 extends LinearOpMode {

    NormalizedColorSensor colorSensor1;
//    NormalizedColorSensor colorSensor2;
//    NormalizedColorSensor colorSensor3;

    // 根據新數據調整的顏色閾值
    private static final float MIN_COLOR_THRESHOLD = 0.005f; // 最小有效信號閾值

    // 紫色和綠色的特徵閾值（基於您提供的數值）
    private static final float PURPLE_BLUE_THRESHOLD = 0.015f;    // 紫色藍色值 > 0.0187
    private static final float PURPLE_GREEN_THRESHOLD = 0.012f;   // 紫色綠色值 > 0.0144
    private static final float GREEN_GREEN_THRESHOLD = 0.012f;    // 綠色綠色值 > 0.0131
    private static final float GREEN_BLUE_THRESHOLD = 0.012f;     // 綠色藍色值 > 0.0134

    // 顏色檢測結果
    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    // 目標顏色模式
    public enum ColorMode {
        GREEN_PURPLE_PURPLE,  // A按鈕
        PURPLE_GREEN_PURPLE,  // B按鈕
        PURPLE_PURPLE_GREEN   // X按鈕
    }

    private ColorMode currentColorMode = ColorMode.GREEN_PURPLE_PURPLE;

    public void init(HardwareMap hwMap) {
        // 初始化三個顏色傳感器
        colorSensor1 = hwMap.get(NormalizedColorSensor.class, "colorSensor1");
//        colorSensor2 = hwMap.get(NormalizedColorSensor.class, "colorSensor2");
//        colorSensor3 = hwMap.get(NormalizedColorSensor.class, "colorSensor3");
    }

    // 更新的檢測方法 - 基於您的新數值
    public DetectedColor getDetectedColor(NormalizedColorSensor sensor, String sensorName, Telemetry telemetry) {
        NormalizedRGBA color = sensor.getNormalizedColors();

        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        telemetry.addData(sensorName + " - R", "%.4f", red);
        telemetry.addData(sensorName + " - G", "%.4f", green);
        telemetry.addData(sensorName + " - B", "%.4f", blue);

        // 檢查信號強度是否足夠
        float maxValue = Math.max(Math.max(red, green), blue);
        if (maxValue < MIN_COLOR_THRESHOLD) {
            telemetry.addData(sensorName + " Status", "Signal too weak");
            return DetectedColor.UNKNOWN;
        }

        // 根據您提供的新數值調整判斷邏輯：
        // 紫色: R=0.0087, G=0.0144, B=0.0187 -> 藍色最高，綠色次之
        // 綠色: R=0.008, G=0.0131, B=0.0134 -> 綠色和藍色接近，但綠色略高

        // 方法1: 絕對值比較
        boolean isPurpleByValue = (blue > PURPLE_BLUE_THRESHOLD) &&
                (green > PURPLE_GREEN_THRESHOLD) &&
                (blue > green);

        boolean isGreenByValue = (green > GREEN_GREEN_THRESHOLD) &&
                (blue > GREEN_BLUE_THRESHOLD) &&
                (green >= blue); // 綠色等於或略高於藍色

        // 方法2: 比率比較（更可靠）
        float total = red + green + blue;
        if (total > 0.01f) { // 確保總和足夠大
            float blueRatio = blue / total;
            float greenRatio = green / total;

            telemetry.addData(sensorName + " - Blue %", "%.1f%%", blueRatio * 100);
            telemetry.addData(sensorName + " - Green %", "%.1f%%", greenRatio * 100);

            // 紫色: 藍色比例較高 (35-45%)
            boolean isPurpleByRatio = (blueRatio > 0.35f) && (blueRatio > greenRatio);
            // 綠色: 綠色和藍色比例接近，但綠色略高 (30-40%)
            boolean isGreenByRatio = (greenRatio > 0.30f) && (greenRatio >= blueRatio) && (greenRatio - blueRatio < 0.1f);

            if (isPurpleByRatio && isPurpleByValue) {
                telemetry.addData(sensorName + " Result", "PURPLE (Both methods agree)");
                return DetectedColor.PURPLE;
            } else if (isGreenByRatio && isGreenByValue) {
                telemetry.addData(sensorName + " Result", "GREEN (Both methods agree)");
                return DetectedColor.GREEN;
            }
        }

        // 方法3: 簡單的最大值判斷（備用）
        if (blue > green && blue > red && blue > 0.015f) {
            telemetry.addData(sensorName + " Result", "PURPLE (Blue is highest)");
            return DetectedColor.PURPLE;
        } else if (green > blue && green > red && green > 0.012f) {
            telemetry.addData(sensorName + " Result", "GREEN (Green is highest)");
            return DetectedColor.GREEN;
        } else if (Math.abs(green - blue) < 0.002f && green > 0.012f) {
            // 綠色和藍色非常接近，但根據您的數據，綠色物體時綠色略高
            telemetry.addData(sensorName + " Result", "GREEN (Green ≈ Blue, but Green slightly higher)");
            return DetectedColor.GREEN;
        }

        telemetry.addData(sensorName + " Result", "UNKNOWN (No clear pattern)");
        return DetectedColor.UNKNOWN;
    }

    // 簡單檢測方法（快速測試用）
    public DetectedColor getDetectedColorSimple(NormalizedColorSensor sensor, String sensorName, Telemetry telemetry) {
        NormalizedRGBA color = sensor.getNormalizedColors();

        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        telemetry.addData(sensorName + " - R", "%.4f", red);
        telemetry.addData(sensorName + " - G", "%.4f", green);
        telemetry.addData(sensorName + " - B", "%.4f", blue);

        // 基於您提供的具體數值：
        // 紫色: B=0.0187, G=0.0144 -> 藍色明顯高於綠色
        // 綠色: G=0.0131, B=0.0134 -> 綠色和藍色非常接近

        float blueGreenDiff = blue - green;

        if (blue > 0.015f && blueGreenDiff > 0.003f) {
            // 藍色值高且明顯大於綠色 -> 紫色
            telemetry.addData(sensorName + " Logic", "PURPLE - Blue significantly higher than green");
            return DetectedColor.PURPLE;
        } else if (green > 0.012f && Math.abs(blueGreenDiff) < 0.001f) {
            // 綠色值高且與藍色非常接近 -> 綠色
            telemetry.addData(sensorName + " Logic", "GREEN - Green and blue are very close");
            return DetectedColor.GREEN;
        } else if (green > 0.012f && blueGreenDiff < 0.001f) {
            // 綠色值高且略高於藍色 -> 綠色
            telemetry.addData(sensorName + " Logic", "GREEN - Green slightly higher than blue");
            return DetectedColor.GREEN;
        }

        return DetectedColor.UNKNOWN;
    }

    // 根據顏色模式分析三個位置的顏色
    private void analyzeColorSequence(Telemetry telemetry) {
        telemetry.addLine("=== COLOR DETECTION START ===");

        // 檢測三個位置的顏色
        DetectedColor color1 = getDetectedColorSimple(colorSensor1, "Sensor1", telemetry);
//        DetectedColor color2 = getDetectedColorSimple(colorSensor2, "Sensor2", telemetry);
//        DetectedColor color3 = getDetectedColorSimple(colorSensor3, "Sensor3", telemetry);

        telemetry.addLine("=== DETECTION RESULTS ===");
        telemetry.addData("Position 1 Color", color1);
//        telemetry.addData("Position 2 Color", color2);
//        telemetry.addData("Position 3 Color", color3);
        telemetry.addData("Current Mode", currentColorMode);

        // 根據當前模式分析
        switch (currentColorMode) {
            case GREEN_PURPLE_PURPLE:
//                analyzeGreenPurplePurple(color1, color2, color3, telemetry);
                break;
            case PURPLE_GREEN_PURPLE:
//                analyzePurpleGreenPurple(color1, color2, color3, telemetry);
                break;
            case PURPLE_PURPLE_GREEN:
//                analyzePurplePurpleGreen(color1, color2, color3, telemetry);
                break;
        }

        // 顯示詳細的數值分析
        displayDetailedAnalysis(telemetry);
    }

    // 分析綠-紫-紫模式
    private void analyzeGreenPurplePurple(DetectedColor c1, DetectedColor c2, DetectedColor c3, Telemetry telemetry) {
        telemetry.addLine("--- GREEN_PURPLE_PURPLE Analysis ---");

        if (c1 == DetectedColor.GREEN) {
            telemetry.addData("✓ MATCH", "Position 1: GREEN (Perfect match!)");
            telemetry.addData("Recommended Action", "ACTIVATE POSITION 1");
        } else if (c2 == DetectedColor.PURPLE) {
            telemetry.addData("✓ MATCH", "Position 2: PURPLE (Good match!)");
            telemetry.addData("Recommended Action", "ACTIVATE POSITION 2");
        } else if (c3 == DetectedColor.PURPLE) {
            telemetry.addData("✓ MATCH", "Position 3: PURPLE (Good match!)");
            telemetry.addData("Recommended Action", "ACTIVATE POSITION 3");
        } else {
            telemetry.addData("✗ NO MATCH", "No perfect match found in current mode");
        }
    }

    // 分析紫-綠-紫模式
    private void analyzePurpleGreenPurple(DetectedColor c1, DetectedColor c2, DetectedColor c3, Telemetry telemetry) {
        telemetry.addLine("--- PURPLE_GREEN_PURPLE Analysis ---");

        if (c1 == DetectedColor.PURPLE) {
            telemetry.addData("✓ MATCH", "Position 1: PURPLE (Perfect match!)");
            telemetry.addData("Recommended Action", "ACTIVATE POSITION 1");
        } else if (c2 == DetectedColor.GREEN) {
            telemetry.addData("✓ MATCH", "Position 2: GREEN (Good match!)");
            telemetry.addData("Recommended Action", "ACTIVATE POSITION 2");
        } else if (c3 == DetectedColor.PURPLE) {
            telemetry.addData("✓ MATCH", "Position 3: PURPLE (Good match!)");
            telemetry.addData("Recommended Action", "ACTIVATE POSITION 3");
        } else {
            telemetry.addData("✗ NO MATCH", "No perfect match found in current mode");
        }
    }

    // 分析紫-紫-綠模式
    private void analyzePurplePurpleGreen(DetectedColor c1, DetectedColor c2, DetectedColor c3, Telemetry telemetry) {
        telemetry.addLine("--- PURPLE_PURPLE_GREEN Analysis ---");

        if (c1 == DetectedColor.PURPLE) {
            telemetry.addData("✓ MATCH", "Position 1: PURPLE (Perfect match!)");
            telemetry.addData("Recommended Action", "ACTIVATE POSITION 1");
        } else if (c2 == DetectedColor.PURPLE) {
            telemetry.addData("✓ MATCH", "Position 2: PURPLE (Good match!)");
            telemetry.addData("Recommended Action", "ACTIVATE POSITION 2");
        } else if (c3 == DetectedColor.GREEN) {
            telemetry.addData("✓ MATCH", "Position 3: GREEN (Good match!)");
            telemetry.addData("Recommended Action", "ACTIVATE POSITION 3");
        } else {
            telemetry.addData("✗ NO MATCH", "No perfect match found in current mode");
        }
    }

    // 顯示詳細的數值分析
    private void displayDetailedAnalysis(Telemetry telemetry) {
        telemetry.addLine("--- DETAILED ANALYSIS ---");

        // 顯示所有傳感器的詳細數值
        NormalizedRGBA color1 = colorSensor1.getNormalizedColors();
//        NormalizedRGBA color2 = colorSensor2.getNormalizedColors();
//        NormalizedRGBA color3 = colorSensor3.getNormalizedColors();

        telemetry.addData("Sensor1 RGB", "R:%.4f G:%.4f B:%.4f", color1.red, color1.green, color1.blue);
//        telemetry.addData("Sensor2 RGB", "R:%.4f G:%.4f B:%.4f", color2.red, color2.green, color2.blue);
//        telemetry.addData("Sensor3 RGB", "R:%.4f G:%.4f B:%.4f", color3.red, color3.green, color3.blue);

        // 計算並顯示藍色-綠色差值
        float diff1 = color1.blue - color1.green;
//        float diff2 = color2.blue - color2.green;
//        float diff3 = color3.blue - color3.green;

        telemetry.addData("Sensor1 B-G Diff", "%.4f", diff1);
//        telemetry.addData("Sensor2 B-G Diff", "%.4f", diff2);
//        telemetry.addData("Sensor3 B-G Diff", "%.4f", diff3);

        // 根據差值給出判斷提示
        telemetry.addLine("--- INTERPRETATION GUIDE ---");
        telemetry.addData("Purple Indicator", "B-G Diff > +0.003");
        telemetry.addData("Green Indicator", "B-G Diff ≈ 0 (±0.001)");
        telemetry.addData("Reference", "Purple: B=0.0187 G=0.0144 | Green: B=0.0134 G=0.0131");
    }

    @Override
    public void runOpMode() {
        init(hardwareMap);

        telemetry.addData("Status", "Initialized - Color Sensor Test Only");
        telemetry.addData("Color Reference", "Purple: R=0.0087 G=0.0144 B=0.0187");
        telemetry.addData("Color Reference", "Green: R=0.0080 G=0.0131 B=0.0134");
        telemetry.addData("Controls", "A/B/X - Change mode, LEFT BUMPER - Detect colors");
        telemetry.addData("Current Mode", currentColorMode);
        telemetry.addData("NOTE", "SERVOS DISABLED - Only displaying results");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 處理按鈕輸入來切換顏色模式
            if (gamepad1.a) {
                currentColorMode = ColorMode.GREEN_PURPLE_PURPLE;
                telemetry.addData("Mode Changed", "GREEN_PURPLE_PURPLE");
                sleep(200);
            } else if (gamepad1.b) {
                currentColorMode = ColorMode.PURPLE_GREEN_PURPLE;
                telemetry.addData("Mode Changed", "PURPLE_GREEN_PURPLE");
                sleep(200);
            } else if (gamepad1.x) {
                currentColorMode = ColorMode.PURPLE_PURPLE_GREEN;
                telemetry.addData("Mode Changed", "PURPLE_PURPLE_GREEN");
                sleep(200);
            }

            // 使用左扳機鍵來觸發顏色檢測和分析
            if (gamepad1.left_bumper) {
                analyzeColorSequence(telemetry);
                sleep(500); // 給時間看結果
            }

            // 顯示當前狀態
            telemetry.addLine("=== CURRENT STATUS ===");
            telemetry.addData("Current Mode", currentColorMode);
            telemetry.addData("Detection", "Press LEFT BUMPER to detect colors");
            telemetry.addLine("");
            telemetry.addData("Control Mapping",
                    "A: Green/Purple/Purple\nB: Purple/Green/Purple\nX: Purple/Purple/Green");

            telemetry.update();
        }
    }
}