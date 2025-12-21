package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "testTeleop_v2")
public class testTeleop_v2 extends LinearOpMode {

    // 顏色感測器
    NormalizedColorSensor colorSensor0;
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    NormalizedColorSensor colorSensor3;

    // Servo
    Servo servo1; // s0
    Servo servo2; // s1
    Servo servo3; // s2

    // 車子移動馬達
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor intakeMotor4;
    DcMotor intakeMotor5;

    // 常數設定
    private static final double SERVO_ACTIVE_POSITION = 0.7;
    private static final double SERVO_REST_POSITION = 0.0;
    private static final int FIRING_INTERVAL_MS = 2000;

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
        colorSensor0 = hwMap.get(NormalizedColorSensor.class, "colorSensor0");
        colorSensor1 = hwMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hwMap.get(NormalizedColorSensor.class, "colorSensor2");
        colorSensor3 = hwMap.get(NormalizedColorSensor.class, "colorSensor3");

        servo1 = hwMap.get(Servo.class, "servo1");
        servo2 = hwMap.get(Servo.class, "servo2");
        servo3 = hwMap.get(Servo.class, "servo3");

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.FORWARD);
        servo3.setDirection(Servo.Direction.REVERSE);

        servo1.scaleRange(0, 0.5);
        servo2.scaleRange(0, 0.5);
        servo3.scaleRange(0, 0.5);


        frontLeftMotor = hwMap.get(DcMotor.class, "motor1");
        frontRightMotor = hwMap.get(DcMotor.class, "motor0");
        backLeftMotor = hwMap.get(DcMotor.class, "motor2");
        backRightMotor = hwMap.get(DcMotor.class, "motor3");
        intakeMotor4 = hwMap.get(DcMotor.class, "motor4");
        intakeMotor5 = hwMap.get(DcMotor.class, "motor5");


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);


        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                servo1.setPosition(SERVO_ACTIVE_POSITION);
                telemetry.addData("Servo Status", "Position 1 (s0) Activated");
                break;
            case 2:
                servo2.setPosition(SERVO_ACTIVE_POSITION);
                telemetry.addData("Servo Status", "Position 2 (s1) Activated");
                break;
            case 3:
                servo3.setPosition(SERVO_ACTIVE_POSITION);
                telemetry.addData("Servo Status", "Position 3 (s2) Activated");
                break;
            default:
                telemetry.addData("Servo Error", "Invalid position: " + position);
                return;
        }

        sleep(300);

        switch (position) {
            case 1: servo1.setPosition(SERVO_REST_POSITION); break;
            case 2: servo2.setPosition(SERVO_REST_POSITION); break;
            case 3: servo3.setPosition(SERVO_REST_POSITION); break;
        }

        telemetry.addData("Servo Action", "Completed for Position " + position);
    }


    private void driveRobot() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y; // Y stick value is reversed
        double rx = gamepad1.right_stick_x;


        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = power * cos/max + rx;
        double frontRightPower = power * sin/max - rx;
        double backLeftPower = power * sin/max + rx;
        double backRightPower = power * cos/max - rx;


        if ((power + Math.abs(rx)) > 1) {
            frontLeftPower /= power + Math.abs(rx);
            frontRightPower /= power + Math.abs(rx);
            backLeftPower /= power + Math.abs(rx);
            backRightPower /= power + Math.abs(rx);
        }

        if(gamepad1.dpad_up){
            intakeMotor4.setPower(1);
            intakeMotor5.setPower(1);
        }
        if(gamepad1.dpad_down){
            intakeMotor4.setPower(0);
            intakeMotor5.setPower(0);
        }




        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    // 顏色感測器相關方法（保持不變）
    private DetectedColor getPositionColor(int position, Telemetry telemetry) {
        switch (position) {
            case 1:
                DetectedColor color1a = getDetectedColorSimple(colorSensor0, "Sensor0", telemetry);
                DetectedColor color1b = getDetectedColorSimple(colorSensor1, "Sensor1", telemetry);
                if (color1a != DetectedColor.UNKNOWN) {
                    return color1a;
                } else if (color1b != DetectedColor.UNKNOWN) {
                    return color1b;
                } else {
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

    // 自動發射序列
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

    // 發射順序決策方法（保持不變）
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

        telemetry.addData("Status", "Initialized - Full Robot Control");
        telemetry.addData("Features", "Mecanum Drive + Color Sensor Auto Firing");
        telemetry.addData("Controls", "Left Stick - Drive, Right Stick - Rotate");
        telemetry.addData("Controls", "A/B/X - Change mode, LEFT BUMPER - Auto fire sequence");
        telemetry.addData("Current Mode", currentColorMode);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. 处理车子移动（每循环都执行，即使在序列进行中）
            driveRobot();

            // 2. 处理模式选择（无论序列是否进行中都可以操作）
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

            // 3. 处理自动发射序列（使用非阻塞方式）
            if (gamepad1.left_bumper && !sequenceInProgress) {
                sequenceInProgress = true;
                // 不等待序列完成，立即继续循环
                new Thread(() -> {
                    executeFullFiringSequence(telemetry);
                    sequenceInProgress = false;
                }).start();
                sleep(200); // 防止重复触发
            }//f

            // 4. 显示状态信息
            telemetry.addLine("=== ROBOT STATUS ===");
            telemetry.addData("Current Mode", currentColorMode);
            telemetry.addData("Sequence Status", sequenceInProgress ? "IN PROGRESS" : "READY");
            telemetry.addData("Drive System", "Mecanum - Active");
            telemetry.addData("Intake System", "D-pad Up/Down to control");
            telemetry.update();
        }

    }
}