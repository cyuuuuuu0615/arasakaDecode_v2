package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "ColorSensorTest_AI_v4")
public class ColorSensorTest_AI_v4 extends LinearOpMode {

    NormalizedColorSensor colorSensor0;
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    NormalizedColorSensor colorSensor3;

    Servo servo1; // s0
    Servo servo2; // s1
    Servo servo3; // s2

    // 根據 Blocks 程式修正位置值
    private static final double SERVO_ACTIVE_POSITION = 0.8;   // 發射位置
    private static final double SERVO_REST_POSITION = 0.0;     // 休息位置 (Blocks中使用0)
    private static final int FIRING_INTERVAL_MS = 300;

    private static final float MIN_COLOR_THRESHOLD = 0.005f;

    public enum DetectedColor {
        PURPLE, GREEN, UNKNOWN
    }

    public enum ColorMode {
        GREEN_PURPLE_PURPLE, PURPLE_GREEN_PURPLE, PURPLE_PURPLE_GREEN
    }

    private ColorMode currentColorMode = ColorMode.GREEN_PURPLE_PURPLE;
    private boolean sequenceInProgress = false;

    public void init(HardwareMap hwMap) {
        // 初始化傳感器
        colorSensor0 = hwMap.get(NormalizedColorSensor.class, "colorSensor0");
        colorSensor1 = hwMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hwMap.get(NormalizedColorSensor.class, "colorSensor2");
        colorSensor3 = hwMap.get(NormalizedColorSensor.class, "colorSensor3");

        // 初始化 Servo 並設定正確方向
        servo1 = hwMap.get(Servo.class, "servo1"); // s0
        servo2 = hwMap.get(Servo.class, "servo2"); // s1
        servo3 = hwMap.get(Servo.class, "servo3"); // s2

        // 根據 Blocks 程式設定方向
        servo1.setDirection(Servo.Direction.REVERSE);  // s0: REVERSE
        servo2.setDirection(Servo.Direction.FORWARD);  // s1: FORWARD
        servo3.setDirection(Servo.Direction.REVERSE);  // s2: REVERSE

        // 設定範圍縮放 (對應 Blocks 的 scaleRange)
        servo1.scaleRange(0, 0.5);
        servo2.scaleRange(0, 0.5);
        servo3.scaleRange(0, 0.5);

        resetAllServos();
    }

    private void resetAllServos() {
        // 設定到休息位置 (0.0)
        servo1.setPosition(SERVO_REST_POSITION);
        servo2.setPosition(SERVO_REST_POSITION);
        servo3.setPosition(SERVO_REST_POSITION);
        sleep(300);
    }

    private void activateServo(int position) {
        telemetry.addData("Servo Action", "Activating Position " + position);

        switch (position) {
            case 1: // s0 (servo1)
                servo1.setPosition(SERVO_ACTIVE_POSITION); // 0.7
                telemetry.addData("Servo Status", "Position 1 (s0) Activated");
                break;
            case 2: // s1 (servo2)
                servo2.setPosition(SERVO_ACTIVE_POSITION); // 0.7
                telemetry.addData("Servo Status", "Position 2 (s1) Activated");
                break;
            case 3: // s2 (servo3)
                servo3.setPosition(SERVO_ACTIVE_POSITION); // 0.7
                telemetry.addData("Servo Status", "Position 3 (s2) Activated");
                break;
            default:
                telemetry.addData("Servo Error", "Invalid position: " + position);
                return;
        }

        // 保持激活狀態 300ms (與 Blocks 一致)
        sleep(300);

        // 回到休息位置 0.0 (與 Blocks 一致)
        switch (position) {
            case 1: servo1.setPosition(SERVO_REST_POSITION); break;
            case 2: servo2.setPosition(SERVO_REST_POSITION); break;
            case 3: servo3.setPosition(SERVO_REST_POSITION); break;
        }

        telemetry.addData("Servo Action", "Completed for Position " + position);
    }

    // 其餘方法保持不變...
    private DetectedColor getPositionColor(int position, Telemetry telemetry) {
        switch (position) {
            case 1:
                DetectedColor color1a = getDetectedColorSimple(colorSensor0, "Sensor0", telemetry);
                DetectedColor color1b = getDetectedColorSimple(colorSensor1, "Sensor1", telemetry);
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
            case 2:
                return getDetectedColorSimple(colorSensor2, "Sensor2", telemetry);
            case 3:
                return getDetectedColorSimple(colorSensor3, "Sensor3", telemetry);
            default:
                return DetectedColor.UNKNOWN;
        }
    }

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
            return DetectedColor.PURPLE;
        } else if (green > 0.012f && Math.abs(blueGreenDiff) < 0.001f) {
            return DetectedColor.GREEN;
        } else if (green > 0.012f && blueGreenDiff < 0.001f) {
            return DetectedColor.GREEN;
        }

        return DetectedColor.UNKNOWN;
    }

    // 自動執行完整發射序列（保持不變）
    private void executeFullFiringSequence(Telemetry telemetry) {
        telemetry.addLine("=== STARTING FULL FIRING SEQUENCE ===");
        telemetry.addData("Mode", currentColorMode);
        telemetry.update();

        // 檢測顏色
        DetectedColor color1 = getPositionColor(1, telemetry);
        DetectedColor color2 = getPositionColor(2, telemetry);
        DetectedColor color3 = getPositionColor(3, telemetry);

        DetectedColor[] colors = {color1, color2, color3};

        telemetry.addData("Position 1 Color", color1);
        telemetry.addData("Position 2 Color", color2);
        telemetry.addData("Position 3 Color", color3);
        telemetry.update();

        // 決定發射順序
        int[] firingOrder = determineFiringOrder(colors, telemetry);

        // 執行發射序列
        telemetry.addLine("--- Executing Firing Sequence ---");
        for (int i = 0; i < firingOrder.length; i++) {
            if (firingOrder[i] != -1) {
                telemetry.addData("Firing", "Position " + firingOrder[i] + " (Step " + (i + 1) + ")");
                telemetry.update();
                activateServo(firingOrder[i]);
                if (i < firingOrder.length - 1) {
                    sleep(FIRING_INTERVAL_MS);
                }
            }
        }

        telemetry.addLine("=== FIRING SEQUENCE COMPLETED ===");
        telemetry.update();
    }

    // 其餘輔助方法保持不變...
    private int[] determineFiringOrder(DetectedColor[] colors, Telemetry telemetry) {
        int[] firingOrder = {-1, -1, -1};
        boolean[] positionUsed = new boolean[3];

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
        return firingOrder;
    }

    private int[] getOrderForGreenPurplePurple(DetectedColor[] colors, boolean[] positionUsed, Telemetry telemetry) {
        int[] order = {-1, -1, -1};
        order[0] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        if (order[0] == -1) order[0] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        order[1] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[1] == -1) order[1] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        order[2] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[2] == -1) order[2] = findAnyAvailablePosition(colors, positionUsed, telemetry);
        return order;
    }

    private int[] getOrderForPurpleGreenPurple(DetectedColor[] colors, boolean[] positionUsed, Telemetry telemetry) {
        int[] order = {-1, -1, -1};
        order[0] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[0] == -1) order[0] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        order[1] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        if (order[1] == -1) order[1] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        order[2] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[2] == -1) order[2] = findAnyAvailablePosition(colors, positionUsed, telemetry);
        return order;
    }

    private int[] getOrderForPurplePurpleGreen(DetectedColor[] colors, boolean[] positionUsed, Telemetry telemetry) {
        int[] order = {-1, -1, -1};
        order[0] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[0] == -1) order[0] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        order[1] = findColorPosition(colors, DetectedColor.PURPLE, positionUsed, telemetry);
        if (order[1] == -1) order[1] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        order[2] = findColorPosition(colors, DetectedColor.GREEN, positionUsed, telemetry);
        if (order[2] == -1) order[2] = findAnyAvailablePosition(colors, positionUsed, telemetry);
        return order;
    }

    private int findColorPosition(DetectedColor[] colors, DetectedColor targetColor, boolean[] positionUsed, Telemetry telemetry) {
        for (int i = 0; i < colors.length; i++) {
            if (!positionUsed[i] && colors[i] == targetColor) {
                positionUsed[i] = true;
                return i + 1;
            }
        }
        return -1;
    }

    private int findAnyAvailablePosition(DetectedColor[] colors, boolean[] positionUsed, Telemetry telemetry) {
        for (int i = 0; i < colors.length; i++) {
            if (!positionUsed[i] && (colors[i] == DetectedColor.GREEN || colors[i] == DetectedColor.PURPLE)) {
                positionUsed[i] = true;
                return i + 1;
            }
        }
        for (int i = 0; i < colors.length; i++) {
            if (!positionUsed[i]) {
                positionUsed[i] = true;
                return i + 1;
            }
        }
        return -1;
    }

    @Override
    public void runOpMode() {
        init(hardwareMap);

        telemetry.addData("Status", "Initialized - Corrected Servo Settings");
        telemetry.addData("Servo Mapping", "s0=servo1(REVERSE), s1=servo2(FORWARD), s2=servo3(REVERSE)");
        telemetry.addData("Servo Positions", "Active: 0.7, Rest: 0.0");
        telemetry.addData("Controls", "A/B/X - Change mode, LEFT BUMPER - Auto fire sequence");
        telemetry.addData("Current Mode", currentColorMode);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
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

            if (gamepad1.left_bumper && !sequenceInProgress) {
                sequenceInProgress = true;
                executeFullFiringSequence(telemetry);
                sequenceInProgress = false;
                sleep(500);
            }

            telemetry.addLine("=== SYSTEM STATUS ===");
            telemetry.addData("Current Mode", currentColorMode);
            telemetry.addData("Sequence Ready", !sequenceInProgress ? "YES" : "IN PROGRESS");
            telemetry.update();
        }

        resetAllServos();
    }
}