package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "testTeleop_v3")
public class testTeleop_v3 extends LinearOpMode {

    NormalizedColorSensor colorSensor0; // servo1 - sensor 1
    NormalizedColorSensor colorSensor1; // servo1 - sensor 2
    NormalizedColorSensor colorSensor2; // servo2 - sensor 1
    NormalizedColorSensor colorSensor3; // servo2 - sensor 2
    NormalizedColorSensor colorSensor4; // servo3 - sensor 1
    NormalizedColorSensor colorSensor5; // servo3 - sensor 2

    // Servo
    Servo servo1; // s0
    Servo servo2; // s1
    Servo servo3; // s2

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor intakeMotor4;
    DcMotor intakeMotor5;

    private static final double SERVO_ACTIVE_POSITION = 0.7;
    private static final double SERVO_REST_POSITION = 0.0;
    private static final int FIRING_INTERVAL_MS = 2000;

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
        colorSensor4 = hwMap.get(NormalizedColorSensor.class, "colorSensor4");
        colorSensor5 = hwMap.get(NormalizedColorSensor.class, "colorSensor5");

        servo1 = hwMap.get(Servo.class, "servo1");
        servo2 = hwMap.get(Servo.class, "servo2");
        servo3 = hwMap.get(Servo.class, "servo3");

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.FORWARD);
        servo3.setDirection(Servo.Direction.REVERSE);

        servo1.scaleRange(0, 0.5);
        servo2.scaleRange(0, 1);
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

        sleep(800);

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
            intakeMotor5.setPower(-1);
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

    private DetectedColor getPositionColor(int position, Telemetry telemetry) {
        switch (position) {
            case 1:
                // Servo1 使用 sensor0 和 sensor1
                return getCombinedColor(colorSensor0, colorSensor1, "Position1", telemetry);
            case 2:
                // Servo2 使用 sensor2 和 sensor3
                return getCombinedColor(colorSensor2, colorSensor3, "Position2", telemetry);
            case 3:
                // Servo3 使用 sensor4 和 sensor5
                return getCombinedColor(colorSensor4, colorSensor5, "Position3", telemetry);
            default:
                return DetectedColor.UNKNOWN;
        }
    }


    private DetectedColor getCombinedColor(NormalizedColorSensor sensor1, NormalizedColorSensor sensor2,
                                           String positionName, Telemetry telemetry) {
        DetectedColor color1 = getSingleSensorColor(sensor1, positionName + "-SensorA", telemetry);
        DetectedColor color2 = getSingleSensorColor(sensor2, positionName + "-SensorB", telemetry);

        telemetry.addData(positionName + " - SensorA", color1);
        telemetry.addData(positionName + " - SensorB", color2);

        // 優先返回確定的顏色
        if (color1 == DetectedColor.PURPLE || color2 == DetectedColor.PURPLE) {
            return DetectedColor.PURPLE;
        }
        if (color1 == DetectedColor.GREEN || color2 == DetectedColor.GREEN) {
            return DetectedColor.GREEN;
        }

        // 兩個傳感器都檢測不到顏色 = UNKNOWN
        return DetectedColor.UNKNOWN;
    }


    public DetectedColor getSingleSensorColor(NormalizedColorSensor sensor, String sensorName, Telemetry telemetry) {
        NormalizedRGBA color = sensor.getNormalizedColors();
        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        // 顯示傳感器數據用於調試
        telemetry.addData(sensorName + " - R", "%.4f", red);
        telemetry.addData(sensorName + " - G", "%.4f", green);
        telemetry.addData(sensorName + " - B", "%.4f", blue);

        // 計算顏色特徵
        float blueGreenDiff = blue - green;
        float total = red + green + blue;

        // 計算歸一化的顏色比例
        float redRatio = red / total;
        float greenRatio = green / total;
        float blueRatio = blue / total;

        telemetry.addData(sensorName + " - R%", "%.1f%%", redRatio * 100);
        telemetry.addData(sensorName + " - G%", "%.1f%%", greenRatio * 100);
        telemetry.addData(sensorName + " - B%", "%.1f%%", blueRatio * 100);

        // 檢測紫色：藍色比例高，紅色比例中等
        boolean isPurple = (blueRatio > 0.35f) && (redRatio > 0.25f) && (redRatio < 0.4f) && (blueGreenDiff > 0.05f);

        // 檢測綠色：綠色比例高，藍色和紅色比例較低
        boolean isGreen = (greenRatio > 0.4f) && (blueRatio < 0.35f) && (redRatio < 0.35f) && (blueGreenDiff < -0.02f);

        if (isPurple) {
            return DetectedColor.PURPLE;
        } else if (isGreen) {
            return DetectedColor.GREEN;
        }

        // 不符合 PURPLE 或 GREEN 特徵 = UNKNOWN
        return DetectedColor.UNKNOWN;
    }

    // 自動發射序列
    private void executeFullFiringSequence(Telemetry telemetry) {
        telemetry.addLine("=== STARTING FULL FIRING SEQUENCE ===");
        telemetry.addData("Mode", currentColorMode);
        telemetry.update();

        // 檢測三個位置的顏色
        DetectedColor color1 = getPositionColor(1, telemetry);
        DetectedColor color2 = getPositionColor(2, telemetry);
        DetectedColor color3 = getPositionColor(3, telemetry);

        DetectedColor[] colors = {color1, color2, color3};

        telemetry.addData("Position 1", color1);
        telemetry.addData("Position 2", color2);
        telemetry.addData("Position 3", color3);
        telemetry.update();

        // 決定發射順序
        int[] firingOrder = determineFiringOrder(colors, telemetry);

        // 執行發射序列 - 所有三個位置都會發射
        telemetry.addLine("--- Executing Firing Sequence ---");
        for (int i = 0; i < firingOrder.length; i++) {
            if (firingOrder[i] != -1) {
                DetectedColor currentColor = colors[firingOrder[i] - 1];
                String ballStatus = (currentColor == DetectedColor.UNKNOWN) ? "UNKNOWN" : currentColor.toString() + " BALL";

                telemetry.addData("Firing", "Position " + firingOrder[i] + " (" + ballStatus + ") - Step " + (i + 1));
                telemetry.update();
                activateServo(firingOrder[i]);
                if (i < firingOrder.length - 1) {
                    sleep(FIRING_INTERVAL_MS);
                }
            }
        }

        telemetry.addLine("=== FIRING SEQUENCE COMPLETED ===");
        telemetry.addData("All Positions", "All 3 positions fired");
        telemetry.update();
    }

    // 發射順序決策方法
    private int[] determineFiringOrder(DetectedColor[] colors, Telemetry telemetry) {
        int[] firingOrder = {-1, -1, -1};
        boolean[] positionUsed = new boolean[3];
        int orderIndex = 0;

        telemetry.addLine("--- Determining Firing Order ---");

        // 第一步：優先發射有球的位置，按照當前模式的理想順序
        DetectedColor[] idealOrder = getIdealColorOrder();

        for (DetectedColor targetColor : idealOrder) {
            int position = findColorPosition(colors, targetColor, positionUsed, telemetry);
            if (position != -1) {
                firingOrder[orderIndex++] = position;
                positionUsed[position - 1] = true;
                telemetry.addData("Priority Shot", "Position " + position + " - " + targetColor);
            }
        }

        // 第二步：發射所有剩餘的位置（包括 UNKNOWN 的位置）
        for (int i = 0; i < colors.length; i++) {
            if (!positionUsed[i]) {
                firingOrder[orderIndex++] = i + 1;
                positionUsed[i] = true;
                telemetry.addData("Remaining Position", "Position " + (i + 1) + " - " + colors[i]);
            }
        }

        telemetry.addData("Final Firing Order",
                "Step1: %d, Step2: %d, Step3: %d",
                firingOrder[0], firingOrder[1], firingOrder[2]);

        return firingOrder;
    }

    // 獲取當前模式的理想顏色順序
    private DetectedColor[] getIdealColorOrder() {
        switch (currentColorMode) {
            case GREEN_PURPLE_PURPLE:
                return new DetectedColor[]{DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE};
            case PURPLE_GREEN_PURPLE:
                return new DetectedColor[]{DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE};
            case PURPLE_PURPLE_GREEN:
                return new DetectedColor[]{DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN};
            default:
                return new DetectedColor[]{DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN};
        }
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

    @Override
    public void runOpMode() {
        init(hardwareMap);

        telemetry.addData("Status", "Initialized - Full Robot Control");
        telemetry.addData("Features", "Mecanum Drive + Color Sensor Auto Firing");
        telemetry.addData("Sensor Setup", "Each servo has 2 color sensors for robust detection");
        telemetry.addData("Detection Logic", "UNKNOWN when both sensors detect no color");
        telemetry.addData("Controls", "Left Stick - Drive, Right Stick - Rotate");
        telemetry.addData("Controls", "A/B/X - Change mode, LEFT BUMPER - Auto fire sequence");
        telemetry.addData("Firing Strategy", "All 3 positions fire, colored balls have priority");
        telemetry.addData("Current Mode", currentColorMode);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. 處理車子移動（每循環都執行，即使在序列進行中）
            driveRobot();

            // 2. 處理模式選擇（無論序列是否進行中都可以操作）
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

            // 3. 處理自動發射序列（使用非阻塞方式）
            if (gamepad1.left_bumper && !sequenceInProgress) {
                sequenceInProgress = true;
                // 不等待序列完成，立即繼續循環
                new Thread(() -> {
                    executeFullFiringSequence(telemetry);
                    sequenceInProgress = false;
                }).start();
                sleep(200); // 防止重複觸發
            }

            telemetry.addLine("=== ROBOT STATUS ===");
            telemetry.addData("Current Mode", currentColorMode);
            telemetry.addData("Sequence Status", sequenceInProgress ? "IN PROGRESS" : "READY");

            if (!sequenceInProgress) {
                DetectedColor color1 = getPositionColor(1, telemetry);
                DetectedColor color2 = getPositionColor(2, telemetry);
                DetectedColor color3 = getPositionColor(3, telemetry);
                telemetry.addData("Position 1", color1);
                telemetry.addData("Position 2", color2);
                telemetry.addData("Position 3", color3);

                int ballCount = getBallCount(new DetectedColor[]{color1, color2, color3});
                telemetry.addData("Colored Balls", ballCount);
                telemetry.addData("Unknown/Empty", (3 - ballCount));
                telemetry.addData("Firing Strategy", "Will fire all 3 positions");
            }

            telemetry.addData("Drive System", "Mecanum - Active");
            telemetry.addData("Intake System", "D-pad Up/Down to control");
            telemetry.update();
        }
    }

    private int getBallCount(DetectedColor[] colors) {
        int count = 0;
        for (DetectedColor color : colors) {
            if (color == DetectedColor.GREEN || color == DetectedColor.PURPLE) {
                count++;
            }
        }
        return count;
    }
}