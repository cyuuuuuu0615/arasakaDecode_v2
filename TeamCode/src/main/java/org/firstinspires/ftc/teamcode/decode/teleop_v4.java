package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop_v4 dualSensor")
public class teleop_v4 extends LinearOpMode {

    // === 硬件變量 ===
    // [修改] 改為兩個顏色傳感器
    NormalizedColorSensor colorSensor1, colorSensor2;

    Servo kickerServo, diskServo, gateServoL, gateServoR;
    DcMotor intakeMotor, shooterMotor;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // === 參數設定 (維持不變) ===
    private static final double FILL_POS_STEP_1 = 0.0;
    private static final double FILL_POS_STEP_2 = 0.3529;
    private static final double FILL_POS_STEP_3 = 0.7137;

    private static final double FIRE_POS_HOLE_B = 0.0471;
    private static final double FIRE_POS_HOLE_C = 0.4314;
    private static final double FIRE_POS_HOLE_A = 0.8196;

    private static final double KICKER_REST = 0.0;
    private static final double KICKER_EXTEND = 0.8;

    // 時間參數 (ms)
    private static final int TIME_BALL_SETTLE = 800;
    private static final int TIME_DISK_MOVE = 500;
    private static final int TIME_SHOOTER_SPIN = 1000;
    private static final int TIME_KICK_OUT = 300;
    private static final int TIME_KICK_RETRACT = 250;

    private static final double GATE_CLOSED = 0.0;
    private static final double GATE_L_OPEN = 0.6667;
    private static final double GATE_R_OPEN = 0.6902;

    private static final float SENSOR_GAIN = 25.0f;
    private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
    private static final float PURPLE_RATIO_LIMIT = 1.2f;
    private static final double INTAKE_POWER = 1.0;

    // === 狀態機定義 ===
    private enum FillState { IDLE, WAIT_SETTLE, ROTATING, FULL }
    private enum FireState { IDLE, PREPARING, DECIDING, AIMING, KICKING, RETRACTING, RESETTING }

    // === 運行時變量 ===
    private FillState fillState = FillState.IDLE;
    private FireState fireState = FireState.IDLE;

    private long fillTimer = 0;
    private long fireTimer = 0;

    private int currentFillStep = 0;
    private boolean hasBallA = false, hasBallB = false, hasBallC = false;
    private String colorHoleA = "EMPTY", colorHoleB = "EMPTY", colorHoleC = "EMPTY";

    private double targetFirePos = 0;
    private String currentTargetHole = "";

    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Dual-Sensor System Initialized");
        telemetry.addData("Sensors", "CS1 & CS2 Active (Gain: %.0f)", SENSOR_GAIN);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // === 麥克納姆輪底盤控制 (Mecanum Drive) ===
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double theta = Math.atan2(y, x);
            double power = Math.hypot(x,y);
            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double frontLeftPower = power * cos/max + rx;
            double frontRightPower = power * sin/max - rx;
            double backLeftPower = power * sin/max + rx;
            double backRightPower = power * cos/max - rx;

            if ((power + Math.abs(rx)) > 1){
                frontLeftPower   /= power + Math.abs(rx);
                frontRightPower /= power + Math.abs(rx);
                backLeftPower    /= power + Math.abs(rx);
                backRightPower  /= power + Math.abs(rx);
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // === 發射觸發 ===
            if (gamepad1.left_bumper && fireState == FireState.IDLE) {
                if (hasBallA || hasBallB || hasBallC) {
                    fireState = FireState.PREPARING;
                    fireTimer = System.currentTimeMillis();
                    controlGates(false);
                    shooterMotor.setPower(0.8);
                }
            }

            // === 執行狀態機 ===
            runFiringLogic();
            runFillingLogic();
            runIntakeLogic();

            updateTelemetry();
        }
    }

    // === 狀態機邏輯：裝球 ===
    private void runFillingLogic() {
        if (fireState != FireState.IDLE) return;

        if (currentFillStep >= 3) {
            fillState = FillState.FULL;
            return;
        } else if (fillState == FillState.FULL) {
            fillState = FillState.IDLE;
        }

        switch (fillState) {
            case IDLE:
                // [修改] 使用雙傳感器綜合檢測方法
                DetectedColor detectedColor = getDualSensorColor();

                if (detectedColor != DetectedColor.UNKNOWN) {
                    recordBallColor(detectedColor);
                    fillTimer = System.currentTimeMillis();
                    fillState = FillState.WAIT_SETTLE;
                }
                break;

            case WAIT_SETTLE:
                if (System.currentTimeMillis() - fillTimer > TIME_BALL_SETTLE) {
                    moveToNextFillPosition();
                    fillTimer = System.currentTimeMillis();
                    fillState = FillState.ROTATING;
                }
                break;

            case ROTATING:
                if (System.currentTimeMillis() - fillTimer > TIME_DISK_MOVE) {
                    fillState = FillState.IDLE;
                }
                break;

            case FULL:
                break;
        }
    }

    // === 狀態機邏輯：發射 (保持不變) ===
    private void runFiringLogic() {
        switch (fireState) {
            case IDLE: break;
            case PREPARING:
                if (System.currentTimeMillis() - fireTimer > TIME_SHOOTER_SPIN) fireState = FireState.DECIDING;
                break;
            case DECIDING:
                if (!hasBallA && !hasBallB && !hasBallC) {
                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.RESETTING;
                    shooterMotor.setPower(0.0);
                    diskServo.setPosition(FILL_POS_STEP_1);
                } else {
                    selectBestTarget();
                    diskServo.setPosition(targetFirePos);
                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.AIMING;
                }
                break;
            case AIMING:
                if (System.currentTimeMillis() - fireTimer > TIME_DISK_MOVE) {
                    kickerServo.setPosition(KICKER_EXTEND);
                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.KICKING;
                }
                break;
            case KICKING:
                if (System.currentTimeMillis() - fireTimer > TIME_KICK_OUT) {
                    kickerServo.setPosition(KICKER_REST);
                    clearBallStatus(currentTargetHole);
                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.RETRACTING;
                }
                break;
            case RETRACTING:
                if (System.currentTimeMillis() - fireTimer > TIME_KICK_RETRACT) fireState = FireState.DECIDING;
                break;
            case RESETTING:
                if (System.currentTimeMillis() - fireTimer > 600) {
                    controlGates(true);
                    currentFillStep = 0;
                    fireState = FireState.IDLE;
                }
                break;
        }
    }

    private void runIntakeLogic() {
        if (currentFillStep < 3 && fireState == FireState.IDLE) {
            intakeMotor.setPower(INTAKE_POWER);
        } else {
            intakeMotor.setPower(0.0);
        }
    }

    // === 輔助方法 ===
    private void recordBallColor(DetectedColor color) {
        switch (currentFillStep) {
            case 0: colorHoleA = color.toString(); hasBallA = true; break;
            case 1: colorHoleB = color.toString(); hasBallB = true; break;
            case 2: colorHoleC = color.toString(); hasBallC = true; break;
        }
    }

    private void moveToNextFillPosition() {
        if (currentFillStep == 0) { diskServo.setPosition(FILL_POS_STEP_2); currentFillStep = 1; }
        else if (currentFillStep == 1) { diskServo.setPosition(FILL_POS_STEP_3); currentFillStep = 2; }
        else if (currentFillStep == 2) { currentFillStep = 3; }
    }

    private void selectBestTarget() {
        double currentPos = diskServo.getPosition();
        double distA = hasBallA ? Math.abs(currentPos - FIRE_POS_HOLE_A) : 999.0;
        double distB = hasBallB ? Math.abs(currentPos - FIRE_POS_HOLE_B) : 999.0;
        double distC = hasBallC ? Math.abs(currentPos - FIRE_POS_HOLE_C) : 999.0;

        if (distA <= distB && distA <= distC) { targetFirePos = FIRE_POS_HOLE_A; currentTargetHole = "A"; }
        else if (distB <= distA && distB <= distC) { targetFirePos = FIRE_POS_HOLE_B; currentTargetHole = "B"; }
        else { targetFirePos = FIRE_POS_HOLE_C; currentTargetHole = "C"; }
    }

    private void clearBallStatus(String hole) {
        if (hole.equals("A")) { hasBallA = false; colorHoleA = "EMPTY"; }
        if (hole.equals("B")) { hasBallB = false; colorHoleB = "EMPTY"; }
        if (hole.equals("C")) { hasBallC = false; colorHoleC = "EMPTY"; }
    }

    private void controlGates(boolean isOpen) {
        if (isOpen) { gateServoL.setPosition(GATE_L_OPEN); gateServoR.setPosition(GATE_R_OPEN); }
        else { gateServoL.setPosition(GATE_CLOSED); gateServoR.setPosition(GATE_CLOSED); }
    }

    private void initHardware() {
        // [修改] 初始化兩個顏色傳感器
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2"); // 記得在 Config 中添加這個名字

        // 開啟 LED 並設置 Gain (兩個都要設)
        if (colorSensor1 instanceof com.qualcomm.robotcore.hardware.SwitchableLight) ((com.qualcomm.robotcore.hardware.SwitchableLight)colorSensor1).enableLight(true);
        if (colorSensor2 instanceof com.qualcomm.robotcore.hardware.SwitchableLight) ((com.qualcomm.robotcore.hardware.SwitchableLight)colorSensor2).enableLight(true);

        colorSensor1.setGain(SENSOR_GAIN);
        colorSensor2.setGain(SENSOR_GAIN);

        kickerServo = hardwareMap.get(Servo.class, "servo1");
        diskServo = hardwareMap.get(Servo.class, "servo2");
        gateServoL = hardwareMap.get(Servo.class, "servo4");
        gateServoR = hardwareMap.get(Servo.class, "servo5");

        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
        shooterMotor = hardwareMap.get(DcMotor.class, "motor5");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kickerServo.scaleRange(0.0, 0.5);
        gateServoL.setDirection(Servo.Direction.REVERSE);
        gateServoR.setDirection(Servo.Direction.FORWARD);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        kickerServo.setPosition(KICKER_REST);
        diskServo.setPosition(FILL_POS_STEP_1);
        controlGates(true);
        intakeMotor.setPower(0);
        shooterMotor.setPower(0);
    }

    // === [新增] 雙傳感器綜合判斷方法 ===
    // 邏輯：只要任一傳感器看到有效顏色(紫/綠)，就返回該顏色
    private DetectedColor getDualSensorColor() {
        DetectedColor c1 = getDetectedColor(colorSensor1);
        DetectedColor c2 = getDetectedColor(colorSensor2);

        // 如果傳感器1看到了顏色 -> 信任它
        if (c1 != DetectedColor.UNKNOWN) {
            return c1;
        }
        // 如果傳感器1沒看到，但傳感器2看到了 -> 信任它
        if (c2 != DetectedColor.UNKNOWN) {
            return c2;
        }

        // 兩個都沒看到 -> UNKNOWN
        return DetectedColor.UNKNOWN;
    }

    // 單一傳感器分析邏輯 (保持不變)
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

    private void updateTelemetry() {
        telemetry.addLine("=== SYSTEM STATUS ===");
        telemetry.addData("Fill State", fillState);
        telemetry.addData("Fire State", fireState);

        if (fireState != FireState.IDLE) {
            telemetry.addData("Action", "FIRING in progress...");
        } else {
            telemetry.addData("Action", "Intake / Idle");
        }

        telemetry.addLine("\n=== BALLS ===");
        telemetry.addData("A", "[%s] %s", colorHoleA, hasBallA?"●":"○");
        telemetry.addData("B", "[%s] %s", colorHoleB, hasBallB?"●":"○");
        telemetry.addData("C", "[%s] %s", colorHoleC, hasBallC?"●":"○");

        // (可選) 顯示兩個傳感器的即時狀態，方便除錯
        // DetectedColor c1 = getDetectedColor(colorSensor1);
        // DetectedColor c2 = getDetectedColor(colorSensor2);
        // telemetry.addData("Sensors", "1:%s | 2:%s", c1, c2);

        telemetry.update();
    }
}