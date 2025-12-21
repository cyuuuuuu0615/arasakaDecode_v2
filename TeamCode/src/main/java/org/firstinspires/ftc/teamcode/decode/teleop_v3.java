package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop_v3")
public class teleop_v3 extends LinearOpMode {

    // === 硬件變量 ===
    NormalizedColorSensor colorSensor1, colorSensor2;
    Servo kickerServo, diskServo, gateServoL, gateServoR;
    DcMotor intakeMotor, shooterMotor;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // === 參數設定 ===
    // 裝球位置 (Filling)
    private static final double FILL_POS_STEP_1 = 0.0;     // Hole A
    private static final double FILL_POS_STEP_2 = 0.3529;  // Hole B
    private static final double FILL_POS_STEP_3 = 0.7137;  // Hole C

    // 發射位置 (Firing)
    private static final double FIRE_POS_HOLE_B = 0.0471;
    private static final double FIRE_POS_HOLE_C = 0.4314;
    private static final double FIRE_POS_HOLE_A = 0.8196;

    // Kicker
    private static final double KICKER_REST = 0.0;
    private static final double KICKER_EXTEND = 0.8;

    // 時間參數 (ms)
    private static final int TIME_BALL_SETTLE = 800;
    private static final int TIME_DISK_MOVE = 500;
    private static final int TIME_SHOOTER_SPIN = 1000;
    private static final int TIME_KICK_OUT = 300;
    private static final int TIME_KICK_RETRACT = 250;

    // 閘門
    private static final double GATE_CLOSED = 0.0;
    private static final double GATE_L_OPEN = 0.6667;
    private static final double GATE_R_OPEN = 0.6902;

    // 傳感器與馬達
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

        telemetry.addData("Status", "Corrected Logic Ready");
        telemetry.addData("Sequence", "Priority: B -> C -> A");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // === 底盤控制 ===
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

            // === 執行邏輯 ===
            runFiringLogic();
            runFillingLogic();
            runIntakeLogic();

            updateTelemetry();
        }

    }

    // === 裝球邏輯 ===
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

            case FULL: break;
        }
    }

    // === 發射邏輯 (依據您的指示修正順序) ===
    private void runFiringLogic() {
        switch (fireState) {
            case IDLE: break;
            case PREPARING:
                if (System.currentTimeMillis() - fireTimer > TIME_SHOOTER_SPIN) fireState = FireState.DECIDING;
                break;

            case DECIDING:
                // 如果已經沒球了，復位
                if (!hasBallA && !hasBallB && !hasBallC) {
                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.RESETTING;
                    shooterMotor.setPower(0.0);

                    // 發射完 A (0.8196) 後，回到 0.0 (Hole A) 開始 Intake
                    diskServo.setPosition(FILL_POS_STEP_1);
                } else {
                    // 選擇目標
                    selectUserPreferredTarget();

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
                    currentFillStep = 0; // 重置計數，從 Hole A 開始
                    fireState = FireState.IDLE;
                }
                break;
        }
    }

    // [修改] 依照您的指示，當在 0.71 (Hole C Fill) 時，優先順序為 B -> C -> A
    private void selectUserPreferredTarget() {
        // 優先檢查 B
        if (hasBallB) {
            targetFirePos = FIRE_POS_HOLE_B; // 0.0471
            currentTargetHole = "B";
        }
        // 其次檢查 C
        else if (hasBallC) {
            targetFirePos = FIRE_POS_HOLE_C; // 0.4314
            currentTargetHole = "C";
        }
        // 最後檢查 A
        else if (hasBallA) {
            targetFirePos = FIRE_POS_HOLE_A; // 0.8196
            currentTargetHole = "A";
        }

        telemetry.addData("Deciding", "Targeting Hole: %s", currentTargetHole);
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

    private void clearBallStatus(String hole) {
        if (hole.equals("A")) { hasBallA = false; colorHoleA = "EMPTY"; }
        if (hole.equals("B")) { hasBallB = false; colorHoleB = "EMPTY"; }
        if (hole.equals("C")) { hasBallC = false; colorHoleC = "EMPTY"; }
    }

    private void controlGates(boolean isOpen) {
        if (isOpen) { gateServoL.setPosition(GATE_L_OPEN); gateServoR.setPosition(GATE_R_OPEN); }
        else { gateServoL.setPosition(GATE_CLOSED); gateServoR.setPosition(GATE_CLOSED); }
    }

    private DetectedColor getDualSensorColor() {
        DetectedColor c1 = getDetectedColor(colorSensor1);
        DetectedColor c2 = getDetectedColor(colorSensor2);
        if (c1 != DetectedColor.UNKNOWN) return c1;
        if (c2 != DetectedColor.UNKNOWN) return c2;
        return DetectedColor.UNKNOWN;
    }

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

    private void initHardware() {
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");

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

    private void updateTelemetry() {
        telemetry.addLine("=== SYSTEM STATUS ===");
        telemetry.addData("Fill State", fillState);
        telemetry.addData("Fire State", fireState);
        telemetry.addData("Target", currentTargetHole);

        if (fireState != FireState.IDLE) telemetry.addData("Action", "FIRING (Priority: B->C->A)");
        else telemetry.addData("Action", "Intake / Idle");

        telemetry.addLine("\n=== BALLS ===");
        telemetry.addData("A", "[%s] %s", colorHoleA, hasBallA?"●":"○");
        telemetry.addData("B", "[%s] %s", colorHoleB, hasBallB?"●":"○");
        telemetry.addData("C", "[%s] %s", colorHoleC, hasBallC?"●":"○");
        telemetry.update();
    }
}