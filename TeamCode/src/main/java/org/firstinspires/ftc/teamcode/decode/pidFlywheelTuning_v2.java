package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "pidFlywheelTuning_v2")
public class pidFlywheelTuning_v2 extends LinearOpMode {

    // === Limelight 與底盤控制參數 ===
    private Limelight3A limelight;
    private final double TARGET_TX = 8.0;
    private double KP_CHASSIS = 0.016;
    private double KD_CHASSIS = 0.073;
    private final double MAX_POWER_CHASSIS = 0.45;
    private final double MIN_POWER_CHASSIS = 0.06;
    private final double DEADBAND_CHASSIS = 1.0;
    private double lastErrorChassis = 0;

    // === 硬件變量 ===
    NormalizedColorSensor colorSensor1, colorSensor2;
    Servo kickerServo, diskServo, gateServoL, gateServoR, angleServo;
    DcMotor intakeMotor, baseMotor;
    DcMotorEx shooterMotorLeft, shooterMotorRight;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // === 參數設定 (位置與時間) ===
    private static final double FILL_POS_STEP_1 = 0.0;
    private static final double FILL_POS_STEP_2 = 0.3529;
    private static final double FILL_POS_STEP_3 = 0.7137;
    private static final double FIRE_POS_HOLE_B = 0.0471;
    private static final double FIRE_POS_HOLE_C = 0.4314;
    private static final double FIRE_POS_HOLE_A = 0.8196;
    private static final double KICKER_REST = 0.0;
    private static final double KICKER_EXTEND = 0.8;
    private static final int TIME_BALL_SETTLE = 150;
    private static final int TIME_DISK_MOVE_INTAKE = 350;
    private static final int TIME_DISK_MOVE_SHOOTING = 500;
    private static final int TIME_SHOOTER_SPIN = 1000;
    private static final int TIME_KICK_OUT = 300;
    private static final int TIME_KICK_RETRACT = 250;
    private static final double GATE_CLOSED = 0.0;
    private static final double GATE_L_OPEN = 0.6667;
    private static final double GATE_R_OPEN = 0.6902;
    private static final float SENSOR_GAIN = 25.0f;
    private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
    private static final float PURPLE_RATIO_LIMIT = 1.2f;
    private static final double INTAKE_POWER = 0.6;

    // === 狀態機變量 ===
    private enum FillState { IDLE, WAIT_SETTLE, ROTATING, FULL }
    private enum FireState { IDLE, PREPARING, DECIDING, AIMING, KICKING, RETRACTING, RESETTING }

    private FillState fillState = FillState.IDLE;
    private FireState fireState = FireState.IDLE;
    private long fillTimer = 0, fireTimer = 0;
    private int currentFillStep = 0;
    private boolean hasBallA = false, hasBallB = false, hasBallC = false;
    private String colorHoleA = "EMPTY", colorHoleB = "EMPTY", colorHoleC = "EMPTY";
    private double targetFirePos = 0;
    private String currentTargetHole = "";

    // === PIDF 目標速度變量 ===
    private double targetVelocity = 0;

    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }

    @Override
    public void runOpMode() {
        initHardware();

        limelight.pipelineSwitch(0);
        limelight.start();

        long lastInputTime = 0;
        long inputDelay = 200;

        waitForStart();

        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();

            // 1. 發射輪速度控制 (D-pad Left/Right 增減 100)
            if (gamepad1.dpad_left && (currentTime - lastInputTime > inputDelay)) {
                targetVelocity += 100;
                lastInputTime = currentTime;
            }
            else if (gamepad1.dpad_right && (currentTime - lastInputTime > inputDelay)) {
                targetVelocity -= 100;
                lastInputTime = currentTime;
            }

            // 速度限制
            targetVelocity = Math.max(0, Math.min(2800, targetVelocity));

            // --- 主從控制核心邏輯 ---
            // Motor 7 (主) 執行 PIDF 速度閉環
            shooterMotorRight.setVelocity(targetVelocity);

            // Motor 5 (從) 直接抓取 Motor 7 當下的 Power 輸出
            double currentPowerFrom7 = shooterMotorRight.getPower();
            shooterMotorLeft.setPower(currentPowerFrom7);

            // 2. Limelight 自動追蹤邏輯 (Base Motor)
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double error = tx - TARGET_TX;
                double errorChange = error - lastErrorChassis;
                double power = (error * KP_CHASSIS) + (errorChange * KD_CHASSIS);

                if (Math.abs(error) > DEADBAND_CHASSIS) {
                    power += (error > 0) ? MIN_POWER_CHASSIS : -MIN_POWER_CHASSIS;
                    power = Math.max(-MAX_POWER_CHASSIS, Math.min(MAX_POWER_CHASSIS, power));
                    baseMotor.setPower(power);
                } else {
                    baseMotor.setPower(0);
                }
                lastErrorChassis = error;
            } else {
                baseMotor.setPower(0);
                lastErrorChassis = 0;
            }

            // 3. 底盤移動邏輯 (Mecanum)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            frontLeftMotor.setPower((y + x + rx) / denominator);
            backLeftMotor.setPower((y - x + rx) / denominator);
            frontRightMotor.setPower((y - x - rx) / denominator);
            backRightMotor.setPower((y + x - rx) / denominator);

            // 4. 輔助動作
            if (gamepad1.dpad_up) angleServo.setPosition(0);
            if (gamepad1.dpad_down) angleServo.setPosition(0.19);

            if (gamepad1.left_bumper && fireState == FireState.IDLE) {
                if (hasBallA || hasBallB || hasBallC) {
                    fireState = FireState.PREPARING;
                    fireTimer = System.currentTimeMillis();
                    controlGates(false);
                }
            }

            // 5. 狀態機與吸取邏輯
            runFiringLogic();
            runFillingLogic();
            if (gamepad1.y) intakeMotor.setPower(-1); else runIntakeLogic();

            updateTelemetry();
        }
        limelight.stop();
    }

    private void initHardware() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // 感應器與伺服機
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        colorSensor1.setGain(SENSOR_GAIN);
        colorSensor2.setGain(SENSOR_GAIN);

        kickerServo = hardwareMap.get(Servo.class, "servo1");
        diskServo = hardwareMap.get(Servo.class, "servo2");
        angleServo = hardwareMap.get(Servo.class, "servo3");
        gateServoL = hardwareMap.get(Servo.class, "servo4");
        gateServoR = hardwareMap.get(Servo.class, "servo5");

        kickerServo.scaleRange(0.0, 0.5);
        gateServoL.setDirection(Servo.Direction.REVERSE);
        angleServo.setDirection(Servo.Direction.FORWARD);

        // 馬達初始化
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "motor5");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "motor7");
        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
        baseMotor = hardwareMap.get(DcMotor.class, "motor6");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");

        // --- 發射輪 PIDF 核心設定 ---
        PIDFCoefficients pidf = new PIDFCoefficients(70, 0, 0, 15);

        // Motor 7 (Master)
        shooterMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooterMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Motor 5 (Slave) - 接收 M7 的 Power，所以使用 RUN_WITHOUT_ENCODER
        shooterMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 底盤與吸取馬達方向
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        baseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 初始化狀態
        kickerServo.setPosition(KICKER_REST);
        diskServo.setPosition(FILL_POS_STEP_1);
        controlGates(true);
    }

    private void runFillingLogic() {
        if (fireState != FireState.IDLE) return;
        if (currentFillStep >= 3) { fillState = FillState.FULL; return; }

        switch (fillState) {
            case IDLE:
                DetectedColor col = getDualSensorColor();
                if (col != DetectedColor.UNKNOWN) {
                    recordBallColor(col);
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
                if (System.currentTimeMillis() - fillTimer > TIME_DISK_MOVE_INTAKE) fillState = FillState.IDLE;
                break;
            case FULL: break;
        }
    }

    private void runFiringLogic() {
        switch (fireState) {
            case PREPARING:
                if (System.currentTimeMillis() - fireTimer > TIME_SHOOTER_SPIN) fireState = FireState.DECIDING;
                break;
            case DECIDING:
                if (hasBallC) { targetFirePos = FIRE_POS_HOLE_C; currentTargetHole = "C"; switchToAiming(); }
                else if (hasBallB) { targetFirePos = FIRE_POS_HOLE_B; currentTargetHole = "B"; switchToAiming(); }
                else if (hasBallA) { targetFirePos = FIRE_POS_HOLE_A; currentTargetHole = "A"; switchToAiming(); }
                else { fireTimer = System.currentTimeMillis(); fireState = FireState.RESETTING; diskServo.setPosition(FILL_POS_STEP_1); }
                break;
            case AIMING:
                if (System.currentTimeMillis() - fireTimer > TIME_DISK_MOVE_SHOOTING) {
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
                if (System.currentTimeMillis() - fireTimer > 600) { controlGates(true); currentFillStep = 0; fireState = FireState.IDLE; }
                break;
            case IDLE: break;
        }
    }

    private void switchToAiming() {
        diskServo.setPosition(targetFirePos);
        fireTimer = System.currentTimeMillis();
        fireState = FireState.AIMING;
    }

    private void runIntakeLogic() {
        if (currentFillStep < 3 && fireState == FireState.IDLE) intakeMotor.setPower(INTAKE_POWER);
        else intakeMotor.setPower(0.0);
    }

    private void recordBallColor(DetectedColor color) {
        if (currentFillStep == 0) { colorHoleA = color.toString(); hasBallA = true; }
        else if (currentFillStep == 1) { colorHoleB = color.toString(); hasBallB = true; }
        else if (currentFillStep == 2) { colorHoleC = color.toString(); hasBallC = true; }
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
        if (c1 != DetectedColor.UNKNOWN) return c1;
        return getDetectedColor(colorSensor2);
    }

    public DetectedColor getDetectedColor(NormalizedColorSensor sensor) {
        NormalizedRGBA color = sensor.getNormalizedColors();
        if (color.alpha < MIN_DETECT_BRIGHTNESS) return DetectedColor.UNKNOWN;
        if (color.blue > color.green && color.blue > color.red && color.blue > (color.green * PURPLE_RATIO_LIMIT)) return DetectedColor.PURPLE;
        if (color.green > color.red && (color.green >= color.blue || (color.green > color.blue * 0.85f))) return DetectedColor.GREEN;
        return DetectedColor.UNKNOWN;
    }

    private void updateTelemetry() {
        telemetry.addLine("=== MASTER-SLAVE SHOOTER ===");
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Master Vel (M7)", "%.1f", shooterMotorRight.getVelocity());
        telemetry.addData("Slave Vel (M5)", "%.1f", shooterMotorLeft.getVelocity());
        telemetry.addData("Current Power Output", "%.3f", shooterMotorRight.getPower());
        telemetry.addLine("\n=== BALL STATUS ===");
        telemetry.addData("A", "[%s] %s", colorHoleA, hasBallA?"●":"○");
        telemetry.addData("B", "[%s] %s", colorHoleB, hasBallB?"●":"○");
        telemetry.addData("C", "[%s] %s", colorHoleC, hasBallC?"●":"○");
        telemetry.update();
    }
}