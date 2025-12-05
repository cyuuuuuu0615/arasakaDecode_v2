package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "ColorSensorTest_AI_v3")
public class ColorSensorTest_AI_v3 extends LinearOpMode {

    NormalizedColorSensor colorSensor0; // 位置1的傳感器A
    NormalizedColorSensor colorSensor1; // 位置1的傳感器B (與sensor0同位置)
    NormalizedColorSensor colorSensor2; // 位置2的傳感器
    NormalizedColorSensor colorSensor3; // 位置3的傳感器

    Servo servo1; // 位置1的Servo
    Servo servo2; // 位置2的Servo
    Servo servo3; // 位置3的Servo

    private static final double SERVO_UP_POSITION = 0.7;
    private static final double SERVO_REST_POSITION = 0;
    private static final int FIRING_INTERVAL_MS = 500; // 發射間隔0.5秒

    private static final float MIN_COLOR_THRESHOLD = 0.005f;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public enum ColorMode {
        GREEN_PURPLE_PURPLE,  // 先綠 → 再紫 → 再紫
        PURPLE_GREEN_PURPLE,  // 先紫 → 再綠 → 再紫
        PURPLE_PURPLE_GREEN   // 先紫 → 再紫 → 再綠
    }

    private ColorMode currentColorMode = ColorMode.GREEN_PURPLE_PURPLE;
    private boolean sequenceInProgress = false; // 防止重複觸發

    public void init(HardwareMap hwMap) {
        // 初始化傳感器 - 注意：sensor0和sensor1對應同一個位置
        colorSensor0 = hwMap.get(NormalizedColorSensor.class, "colorSensor0");
//        colorSensor1 = hwMap.get(NormalizedColorSensor.class, "colorSensor1");
//        colorSensor2 = hwMap.get(NormalizedColorSensor.class, "colorSensor2");
//        colorSensor3 = hwMap.get(NormalizedColorSensor.class, "colorSensor3");

        // 初始化3個Servo
        servo1 = hwMap.get(Servo.class, "servo1");
        servo2 = hwMap.get(Servo.class, "servo2");
        servo3 = hwMap.get(Servo.class, "servo3");
        servo1.setDirection(Servo.Direction.REVERSE);

        resetAllServos();
    }

    private void resetAllServos() {
        servo1.setPosition(SERVO_REST_POSITION);
        servo2.setPosition(SERVO_REST_POSITION);
        servo3.setPosition(SERVO_REST_POSITION);
        sleep(300);
    }

    private void activateServo(int position) {
        telemetry.addData("Servo Action", "Activating Position " + position);

        switch (position) {
            case 1:
                servo1.setPosition(SERVO_UP_POSITION);
                telemetry.addData("Servo Status", "Position 1 Activated");
                break;
            case 2:
                servo2.setPosition(SERVO_UP_POSITION);
                telemetry.addData("Servo Status", "Position 2 Activated");
                break;
            case 3:
                servo3.setPosition(SERVO_UP_POSITION);
                telemetry.addData("Servo Status", "Position 3 Activated");
                break;
            default:
                telemetry.addData("Servo Error", "Invalid position: " + position);
                return;
        }

        // 保持按下狀態
        sleep(300);

        // 回到休息位置
        switch (position) {
            case 1: servo1.setPosition(SERVO_REST_POSITION); break;
            case 2: servo2.setPosition(SERVO_REST_POSITION); break;
            case 3: servo3.setPosition(SERVO_REST_POSITION); break;
        }

        telemetry.addData("Servo Action", "Completed for Position " + position);
    }

    // 檢測單個位置的顏色（處理雙傳感器）
    private DetectedColor getPositionColor(int position, Telemetry telemetry) {
        switch (position) {
            case 1: // 位置1：使用sensor0和sensor1，任一檢測到即可
                DetectedColor color1a = getDetectedColorSimple(colorSensor0, "Sensor0", telemetry);
                DetectedColor color1b = getDetectedColorSimple(colorSensor1, "Sensor1", telemetry);

                // 雙傳感器邏輯：任一檢測到有效顏色即可
                if (color1a != DetectedColor.UNKNOWN) {
                    telemetry.addData("Position 1", "Using Sensor0 result: " + color1a);
                    return color1a;
                } else if (color1b != DetectedColor.UNKNOWN) {
                    telemetry.addData("Position 1", "Using Sensor1 result: " + color1b);
                    return color1b;
                } else {
                    telemetry.addData("Position 1", "Both sensors show UNKNOWN");
                    return DetectedColor.UNKNOWN;
                }

            case 2: // 位置2：使用sensor2
                return getDetectedColorSimple(colorSensor2, "Sensor2", telemetry);

            case 3: // 位置3：使用sensor3
                return getDetectedColorSimple(colorSensor3, "Sensor3", telemetry);

            default:
                return DetectedColor.UNKNOWN;
        }
    }

    // 簡單檢測方法
    public DetectedColor getDetectedColorSimple(NormalizedColorSensor sensor, String sensorName, Telemetry telemetry) {
        NormalizedRGBA color = sensor.getNormalizedColors();

        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        telemetry.addData(sensorName + " - R", "%.4f", red);
        telemetry.addData(sensorName + " - G", "%.4f", green);
        telemetry.addData(sensorName + " - B", "%.4f", blue);

        float blueGreenDiff = blue - green;

        if (blue > 0.015f && blueGreenDiff > 0.003f) {
            telemetry.addData(sensorName + " Logic", "PURPLE - Blue significantly higher than green");
            return DetectedColor.PURPLE;
        } else if (green > 0.012f && Math.abs(blueGreenDiff) < 0.001f) {
            telemetry.addData(sensorName + " Logic", "GREEN - Green and blue are very close");
            return DetectedColor.GREEN;
        } else if (green > 0.012f && blueGreenDiff < 0.001f) {
            telemetry.addData(sensorName + " Logic", "GREEN - Green slightly higher than blue");
            return DetectedColor.GREEN;
        }

        return DetectedColor.UNKNOWN;
    }

    // 自動執行完整發射序列
    private void executeFullFiringSequence(Telemetry telemetry) {
        telemetry.addLine("=== STARTING FULL FIRING SEQUENCE ===");
        telemetry.addData("Mode", currentColorMode);
        telemetry.addData("Firing Interval", FIRING_INTERVAL_MS + "ms");
        telemetry.update();

        // 步驟1: 檢測所有位置顏色
        telemetry.addLine("--- Step 1: Detecting Colors ---");
        DetectedColor color1 = getPositionColor(1, telemetry);
        DetectedColor color2 = getPositionColor(2, telemetry);
        DetectedColor color3 = getPositionColor(3, telemetry);

        DetectedColor[] colors = {color1, color2, color3};

        telemetry.addData("Position 1 Color", color1);
        telemetry.addData("Position 2 Color", color2);
        telemetry.addData("Position 3 Color", color3);
        telemetry.update();

        // 步驟2: 根據模式決定發射順序
        telemetry.addLine("--- Step 2: Determining Firing Order ---");
        int[] firingOrder = determineFiringOrder(colors, telemetry);

        // 步驟3: 按順序發射三個位置
        telemetry.addLine("--- Step 3: Executing Firing Sequence ---");
        for (int i = 0; i < firingOrder.length; i++) {
            if (firingOrder[i] != -1) {
                telemetry.addData("Firing", "Position " + firingOrder[i] + " (Step " + (i + 1) + ")");
                telemetry.update();

                activateServo(firingOrder[i]);

                // 發射間隔（最後一個不用等待）
                if (i < firingOrder.length - 1) {
                    sleep(FIRING_INTERVAL_MS);
                }
            } else {
                telemetry.addData("Skipping", "No valid position for step " + (i + 1));
                telemetry.update();
            }
        }

        telemetry.addLine("=== FIRING SEQUENCE COMPLETED ===");
        telemetry.addData("Status", "All 3 positions processed");
        telemetry.update();
    }

    // 根據模式和檢測結果決定發射順序
    private int[] determineFiringOrder(DetectedColor[] colors, Telemetry telemetry) {
        int[] firingOrder = {-1, -1, -1}; // 初始化為無效值
        boolean[] positionUsed = new boolean[3]; // 追蹤已使用的位置

        telemetry.addLine("Determining firing order for mode: " + currentColorMode);

        switch (currentColorMode) {
            case GREEN_PURPLE_PURPLE:
                firingOrder = getOrderForGreenPurplePurple(colors, positionUsed, telemetry);
                break;
            case PURPLE_GREEN_PURPLE:
                firingOrder = getOrderForPurpleGreenPurple(colors, positionUsed, telemetry);
                break;
            case PURPLE_PURPLE_GREEN:
                firingOrder = getOrderForPurplePurpleGreen(colors, positionUsed, telemetry);
                break;
        }

        // 顯示最終發射順序
        telemetry.addLine("Final Firing Order:");
        for (int i = 0; i < firingOrder.length; i++) {
            telemetry.addData("Step " + (i + 1),
                    firingOrder[i] != -1 ? "Position " + firingOrder[i] : "SKIP");
        }

        return firingOrder;
    }

    // 綠-紫-紫模式的發射順序
    private int[] getOrderForGreenPurplePurple(DetectedColor[] colors, boolean[] positionUsed, Telemetry telemetry) {
        int[] order = {-1, -1, -1};

        // 步驟1: 找綠色
        order[0] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        if (order[0] == -1) {
            telemetry.addData("Step 1 Fallback", "No GREEN, looking for PURPLE");
            order[0] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        }

        // 步驟2: 找第一個紫色
        order[1] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[1] == -1) {
            telemetry.addData("Step 2 Fallback", "No PURPLE, looking for GREEN");
            order[1] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        }

        // 步驟3: 找第二個紫色或任何可用位置
        order[2] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[2] == -1) {
            telemetry.addData("Step 3 Fallback", "No PURPLE, looking for any position");
            order[2] = findAnyAvailablePosition(colors, positionUsed, telemetry);
        }

        return order;
    }

    // 紫-綠-紫模式的發射順序
    private int[] getOrderForPurpleGreenPurple(DetectedColor[] colors, boolean[] positionUsed, Telemetry telemetry) {
        int[] order = {-1, -1, -1};

        // 步驟1: 找第一個紫色
        order[0] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[0] == -1) {
            telemetry.addData("Step 1 Fallback", "No PURPLE, looking for GREEN");
            order[0] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        }

        // 步驟2: 找綠色
        order[1] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        if (order[1] == -1) {
            telemetry.addData("Step 2 Fallback", "No GREEN, looking for PURPLE");
            order[1] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        }

        // 步驟3: 找第二個紫色或任何可用位置
        order[2] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[2] == -1) {
            telemetry.addData("Step 3 Fallback", "No PURPLE, looking for any position");
            order[2] = findAnyAvailablePosition(colors, positionUsed, telemetry);
        }

        return order;
    }

    // 紫-紫-綠模式的發射順序
    private int[] getOrderForPurplePurpleGreen(DetectedColor[] colors, boolean[] positionUsed, Telemetry telemetry) {
        int[] order = {-1, -1, -1};

        // 步驟1: 找第一個紫色
        order[0] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[0] == -1) {
            telemetry.addData("Step 1 Fallback", "No PURPLE, looking for GREEN");
            order[0] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        }

        // 步驟2: 找第二個紫色
        order[1] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[1] == -1) {
            telemetry.addData("Step 2 Fallback", "No PURPLE, looking for GREEN");
            order[1] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        }

        // 步驟3: 找綠色或任何可用位置
        order[2] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        if (order[2] == -1) {
            telemetry.addData("Step 3 Fallback", "No GREEN, looking for any position");
            order[2] = findAnyAvailablePosition(colors, positionUsed, telemetry);
        }

        return order;
    }

    // 尋找指定顏色的位置
    private int findColorPosition(DetectedColor[] colors, DetectedColor targetColor, boolean[] positionUsed, Telemetry telemetry) {
        for (int i = 0; i < colors.length; i++) {
            if (!positionUsed[i] && colors[i] == targetColor) {
                positionUsed[i] = true;
                telemetry.addData("Found " + targetColor, "at position " + (i + 1));
                return i + 1;
            }
        }
        telemetry.addData("Not Found", "No available " + targetColor + " ball detected");
        return -1;
    }

    // 尋找任何可用的位置
    private int findAnyAvailablePosition(DetectedColor[] colors, boolean[] positionUsed, Telemetry telemetry) {
        // 先嘗試找綠色或紫色
        for (int i = 0; i < colors.length; i++) {
            if (!positionUsed[i] && (colors[i] == DetectedColor.GREEN || colors[i] == DetectedColor.PURPLE)) {
                positionUsed[i] = true;
                telemetry.addData("Found Available", colors[i] + " ball at position " + (i + 1));
                return i + 1;
            }
        }

        // 如果都沒有，找第一個未使用的位置
        for (int i = 0; i < colors.length; i++) {
            if (!positionUsed[i]) {
                positionUsed[i] = true;
                telemetry.addData("Found Any", "Position " + (i + 1) + " (color: " + colors[i] + ")");
                return i + 1;
            }
        }

        telemetry.addData("No Available", "All positions already used");
        return -1;
    }

    // 顯示詳細分析
    private void displayColorAnalysis(Telemetry telemetry) {
        telemetry.addLine("=== CURRENT COLOR STATUS ===");

        // 顯示位置1的雙傳感器數據
        NormalizedRGBA color0 = colorSensor0.getNormalizedColors();
        NormalizedRGBA color1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA color2 = colorSensor2.getNormalizedColors();
        NormalizedRGBA color3 = colorSensor3.getNormalizedColors();

        telemetry.addData("Position1-Sensor0", "R:%.4f G:%.4f B:%.4f", color0.red, color0.green, color0.blue);
        telemetry.addData("Position1-Sensor1", "R:%.4f G:%.4f B:%.4f", color1.red, color1.green, color1.blue);
        telemetry.addData("Position2-Sensor2", "R:%.4f G:%.4f B:%.4f", color2.red, color2.green, color2.blue);
        telemetry.addData("Position3-Sensor3", "R:%.4f G:%.4f B:%.4f", color3.red, color3.green, color3.blue);
    }

    @Override
    public void runOpMode() {
        init(hardwareMap);

        telemetry.addData("Status", "Initialized - Color Sensor Test v8 with Auto Sequence");
        telemetry.addData("New Feature", "Press LEFT BUMPER once for automatic 3-step firing sequence");
        telemetry.addData("Firing Interval", FIRING_INTERVAL_MS + "ms between shots");
        telemetry.addData("Sensor Layout", "Position1: Dual sensors, Position2/3: Single sensors");
        telemetry.addData("Controls", "A/B/X - Change mode, LEFT BUMPER - Auto fire sequence, Y - Reset servos");
        telemetry.addData("Current Mode", currentColorMode);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 模式選擇
            if (gamepad1.a) {
                currentColorMode = ColorMode.GREEN_PURPLE_PURPLE;
                sequenceInProgress = false;
                telemetry.addData("Mode Changed", "GREEN_PURPLE_PURPLE");
                sleep(200);
            } else if (gamepad1.b) {
                currentColorMode = ColorMode.PURPLE_GREEN_PURPLE;
                sequenceInProgress = false;
                telemetry.addData("Mode Changed", "PURPLE_GREEN_PURPLE");
                sleep(200);
            } else if (gamepad1.x) {
                currentColorMode = ColorMode.PURPLE_PURPLE_GREEN;
                sequenceInProgress = false;
                telemetry.addData("Mode Changed", "PURPLE_PURPLE_GREEN");
                sleep(200);
            }

            // 執行完整發射序列（防止重複觸發）
            if (gamepad1.left_bumper && !sequenceInProgress) {
                sequenceInProgress = true;
                executeFullFiringSequence(telemetry);
                sequenceInProgress = false;
                sleep(500); // 防止立即重複觸發
            }

            // 手動重置Servo
            if (gamepad1.y) {
                resetAllServos();
                telemetry.addData("Manual Reset", "All servos reset to rest position");
                sleep(200);
            }

            // 顯示當前狀態
            telemetry.addLine("=== SYSTEM STATUS ===");
            telemetry.addData("Current Mode", currentColorMode);
            telemetry.addData("Sequence Ready", !sequenceInProgress ? "YES" : "IN PROGRESS");
            telemetry.addData("Next Action", "Press LEFT BUMPER for auto firing sequence");
            telemetry.addData("Reset", "Press Y to reset servos");

            displayColorAnalysis(telemetry);
            telemetry.update();
        }

        resetAllServos();
    }
}