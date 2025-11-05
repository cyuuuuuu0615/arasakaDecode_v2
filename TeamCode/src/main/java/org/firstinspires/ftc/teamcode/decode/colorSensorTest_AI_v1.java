package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "colorSensorTest_AI_v1")
public class colorSensorTest_AI_v1 extends LinearOpMode {

    NormalizedColorSensor colorSensor;
    Servo testServo;

    // 根據您的新數據調整的極敏感閾值
    private static final float MIN_COLOR_THRESHOLD = 0.0005f; // 非常低的閾值

    // Servo 位置
    private static final double SERVO_UP_POSITION = 0.8;
    private static final double SERVO_DOWN_POSITION = 0.2;

    private boolean servoActivated = false;
    private int detectionCount = 0;
    private static final int REQUIRED_DETECTIONS = 3; // 需要連續檢測到多次才觸發

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        testServo = hwMap.get(Servo.class, "testServo");
        testServo.setPosition(SERVO_UP_POSITION);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();

        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        telemetry.addData("Red", "%.4f", red); // 顯示4位小數
        telemetry.addData("Green", "%.4f", green);
        telemetry.addData("Blue", "%.4f", blue);

        // 檢查數據是否有效
        if (red < MIN_COLOR_THRESHOLD && green < MIN_COLOR_THRESHOLD && blue < MIN_COLOR_THRESHOLD) {
            telemetry.addData("Status", "Signal too weak");
            return DetectedColor.UNKNOWN;
        }

        // 修正：交換綠色和紫色的判斷邏輯
        // 紫色物體的藍色值最高
        if (blue > 0.0035f && blue > green && blue > red) {
            telemetry.addData("Logic", "PURPLE detected - Blue is highest");
            return DetectedColor.PURPLE;
        }
        // 綠色物體的綠色值最高
        else if (green > 0.0025f && green > blue && green > red) {
            telemetry.addData("Logic", "GREEN detected - Green is highest");
            return DetectedColor.GREEN;
        }

        return DetectedColor.UNKNOWN;
    }

    // 更精確的比率方法 - 修正版本
    public DetectedColor getDetectedColorByRatio(Telemetry telemetry) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();

        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        telemetry.addData("Red", "%.4f", red);
        telemetry.addData("Green", "%.4f", green);
        telemetry.addData("Blue", "%.4f", blue);

        // 計算總和
        float sum = red + green + blue;
        if (sum < 0.002f) { // 總和太小
            telemetry.addData("Status", "Signal too weak - sum: %.4f", sum);
            return DetectedColor.UNKNOWN;
        }

        // 計算比率
        float redRatio = red / sum;
        float greenRatio = green / sum;
        float blueRatio = blue / sum;

        telemetry.addData("Red %", "%.1f%%", redRatio * 100);
        telemetry.addData("Green %", "%.1f%%", greenRatio * 100);
        telemetry.addData("Blue %", "%.1f%%", blueRatio * 100);

        // 修正：交換綠色和紫色的比率判斷
        // 紫色物體：藍色比例 > 45%
        if (blueRatio > 0.45f && blueRatio > greenRatio && blueRatio > redRatio) {
            telemetry.addData("Logic", "PURPLE - Blue ratio: %.1f%%", blueRatio * 100);
            return DetectedColor.PURPLE;
        }
        // 綠色物體：綠色比例 > 40%
        else if (greenRatio > 0.40f && greenRatio > blueRatio && greenRatio > redRatio) {
            telemetry.addData("Logic", "GREEN - Green ratio: %.1f%%", greenRatio * 100);
            return DetectedColor.GREEN;
        }

        return DetectedColor.UNKNOWN;
    }

    // 增強檢測方法 - 使用多種條件（修正版本）
    public DetectedColor getDetectedColorEnhanced(Telemetry telemetry) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();

        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        telemetry.addData("Raw - R", "%.4f", red);
        telemetry.addData("Raw - G", "%.4f", green);
        telemetry.addData("Raw - B", "%.4f", blue);

        // 修正：交換條件檢測
        boolean possiblePurple = (blue > 0.003f) && (blue > green) && (blue > red);
        boolean possibleGreen = (green > 0.002f) && (green > blue) && (green > red);

        // 檢查信號強度
        float maxVal = Math.max(Math.max(red, green), blue);
        boolean hasGoodSignal = maxVal > 0.002f;

        telemetry.addData("Max Value", "%.4f", maxVal);
        telemetry.addData("Good Signal", hasGoodSignal);
        telemetry.addData("Possible Purple", possiblePurple);
        telemetry.addData("Possible Green", possibleGreen);

        if (hasGoodSignal) {
            if (possiblePurple && !possibleGreen) {
                return DetectedColor.PURPLE;
            } else if (possibleGreen && !possiblePurple) {
                return DetectedColor.GREEN;
            }
        }

        return DetectedColor.UNKNOWN;
    }

    // 簡單檢測方法 - 修正版本
    public DetectedColor getDetectedColorSimple(Telemetry telemetry) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();

        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        telemetry.addData("R", "%.4f", red);
        telemetry.addData("G", "%.4f", green);
        telemetry.addData("B", "%.4f", blue);

        // 最簡單的邏輯：哪個顏色值最高就判斷為什麼顏色（修正版本）
        float maxValue = Math.max(Math.max(red, green), blue);

        // 修正：交換判斷條件
        if (maxValue == blue && blue > 0.003f) {
            telemetry.addData("Logic", "PURPLE - Blue is highest");
            return DetectedColor.PURPLE;
        }
        else if (maxValue == green && green > 0.002f) {
            telemetry.addData("Logic", "GREEN - Green is highest");
            return DetectedColor.GREEN;
        }

        return DetectedColor.UNKNOWN;
    }

    public void moveServoForPurple() {
        detectionCount++;
        telemetry.addData("Detection Count", "%d/%d", detectionCount, REQUIRED_DETECTIONS);

        if (!servoActivated && detectionCount >= REQUIRED_DETECTIONS) {
            testServo.setPosition(SERVO_DOWN_POSITION);
            telemetry.addData("Servo", "Moving DOWN - PURPLE DETECTED");
            servoActivated = true;
        }
    }

    public void resetDetection() {
        detectionCount = 0;
        if (servoActivated) {
            testServo.setPosition(SERVO_UP_POSITION);
            telemetry.addData("Servo", "Moving UP");
            servoActivated = false;
        }
    }

    @Override
    public void runOpMode() {
        init(hardwareMap);

        telemetry.addData("Status", "Initialized - Color Logic Corrected");
        telemetry.addData("Detection Logic", "High Blue = PURPLE, High Green = GREEN");
        telemetry.addData("Controls", "A - Reset, X - Test Servo, Y - Switch Method");
        telemetry.update();

        int detectionMethod = 0; // 0: Enhanced, 1: Simple, 2: Ratio

        waitForStart();

        while (opModeIsActive()) {
            DetectedColor detectedColor;

            // 根據選擇的方法進行檢測
            switch (detectionMethod) {
                case 0:
                    detectedColor = getDetectedColorEnhanced(telemetry);
                    telemetry.addData("Method", "Enhanced");
                    break;
                case 1:
                    detectedColor = getDetectedColorSimple(telemetry);
                    telemetry.addData("Method", "Simple");
                    break;
                case 2:
                    detectedColor = getDetectedColorByRatio(telemetry);
                    telemetry.addData("Method", "Ratio");
                    break;
                default:
                    detectedColor = getDetectedColorEnhanced(telemetry);
                    telemetry.addData("Method", "Enhanced (default)");
                    break;
            }

            telemetry.addData("Detected Color", detectedColor);

            // 只有檢測到紫色時才動作 servo
            if (detectedColor == DetectedColor.PURPLE) {
                moveServoForPurple();
            } else {
                resetDetection();
            }

            // 手動控制
            if (gamepad1.a) {
                resetDetection();
            }
            if (gamepad1.x) {
                testServo.setPosition(SERVO_DOWN_POSITION);
                sleep(500);
                testServo.setPosition(SERVO_UP_POSITION);
            }
            if (gamepad1.y) {
                detectionMethod = (detectionMethod + 1) % 3;
                telemetry.addData("Method Changed", "New method: " + detectionMethod);
                sleep(300); // 防止快速切換
            }

            telemetry.update();
            sleep(100);
        }
    }
}