package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

// [修改] 名稱改為 RED
@TeleOp(name = "teleop_v5")
public class teleop_v5 extends LinearOpMode {

    // === Limelight 與 PD 控制參數 ===
    private Limelight3A limelight;

    // [修改] 紅隊的 Target Offset 與藍隊相反
    // 藍隊是 6.0，紅隊改為 -6.0
    private final double TARGET_TX = -6.0;

    private final double MIN_POWER = 0.06;
    private double lastError = 0;
    private double KP = 0.011;
    private double KD = 0.095;
    private final double MAX_POWER = 0.40;
    private final double DEADBAND = 1.5;

    // ==========================================
    // === 距離與速度計算參數 ===
    // ==========================================
    private static final double CAMERA_HEIGHT = 14.5;
    private static final double TARGET_HEIGHT = 39.0;
    private static final double MOUNT_ANGLE = 17.8;
    private static final double DISTANCE_THRESHOLD = 35.0;
    private static final double ANGLE_CLOSE = 0.0;
    private static final double ANGLE_FAR = 0.12;

    // --- RPM 參數 ---
    private static final double RPM_SLOPE_CLOSE = 11.0;
    private static final double RPM_BASE_CLOSE = 610.0;
    private static final double RPM_SLOPE_FAR = 10.0;
    private static final double RPM_BASE_FAR = 620.0;
    private static final double RPM_IDLE = 300.0;

    // === 緩降邏輯變數 ===
    private double currentCommandedRpm = RPM_IDLE;
    private static final double RPM_RAMP_DOWN_STEP = 5.0;

    // === 硬件變量 ===
    NormalizedColorSensor colorSensor1, colorSensor2;
    Servo kickerServo, diskServo, gateServoL, gateServoR, angleServo;
    DcMotor intakeMotor, baseMotor;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    DcMotorEx shooterMotorLeft, shooterMotorRight;
    private LED LED0, LED1, LED2;

    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(92, 0, 0, 15);

    // === 其他常數 ===
    private static final double FILL_POS_STEP_1 = 0.0;    // Hole A
    private static final double FILL_POS_STEP_2 = 0.3529; // Hole B
    private static final double FILL_POS_STEP_3 = 0.7137; // Hole C

    private static final double FIRE_POS_HOLE_B = 0.0471;
    private static final double FIRE_POS_HOLE_C = 0.4314;
    private static final double FIRE_POS_HOLE_A = 0.8196;

    private static final double KICKER_REST = 0.0;
    private static final double KICKER_EXTEND = 0.8;

    // 時間參數
    private static final int TIME_BALL_SETTLE = 170;
    private static final int TIME_DISK_MOVE_INTAKE = 350;
    private static final int TIME_DISK_MOVE_SHOOTING = 600;
    private static final int TIME_SHOOTER_SPIN = 1000;
    private static final int TIME_KICK_OUT = 300;
    private static final int TIME_KICK_RETRACT = 250;

    private static final double GATE_CLOSED = 0.0;
    private static final double GATE_L_OPEN = 0.6667;
    private static final double GATE_R_OPEN = 0.6902;

    private static final float SENSOR_GAIN = 25.0f;
    private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
    private static final float PURPLE_RATIO_LIMIT = 1.2f;
    private static final double INTAKE_POWER = 1;

    private enum FillState { IDLE, WAIT_SETTLE, ROTATING, FULL }
    private enum FireState { IDLE, PREPARING, DECIDING, AIMING, KICKING, RETRACTING, RESETTING }
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

    private double desiredTargetRpm = 0;
    private boolean isHighSpeedMode = false;
    private boolean lastRightBumper = false;

    // 用於按鈕 A 的防手抖變數
    private boolean lastButtonA = false;

    @Override
    public void runOpMode() {
        initHardware();

        try {
            // 注意：如果你的 Limelight 設定中，Pipeline 0 是給藍隊，Pipeline 1 是給紅隊
            // 你可能需要把這裡改成 limelight.pipelineSwitch(1);
            // 如果你是在同一個 Pipeline 0 修改設定來追蹤 ID 24，則保持不變。
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            telemetry.addData("Limelight Error", "Init Failed: " + e.getMessage());
        }

        angleServo.setDirection(Servo.Direction.REVERSE);
        angleServo.setPosition(ANGLE_CLOSE);

        telemetry.addData("Status", "RED ALLIANCE Ready (Target TX: " + TARGET_TX + ")");
        telemetry.update();

        long lastInputTime = 0;
        long inputDelay = 200;

        waitForStart();

        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();

            LLResult result = null;
            boolean targetValid = false;
            try {
                result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    // 這裡可以選擇性加入判斷，確保追蹤的是 ID 24 (如果 Limelight 看得到多個 Tag)
                    // 例如: if (result.getTa() > 0) ...
                    targetValid = true;
                }
            } catch (Exception e) { targetValid = false; }

            // =======================================================
            // 1. BaseMotor (砲塔) 自動控制
            // =======================================================
            if (targetValid) {
                double tx = result.getTx();

                // 計算誤差：目標是 -6.0 (RED)
                // Error = 當前 tx - (-6.0) = tx + 6.0
                double error = tx - TARGET_TX;

                double errorChange = error - lastError;
                double dTerm = errorChange * KD;

                if (Math.abs(error) > DEADBAND) {
                    double power = (error * KP) + dTerm;

                    // 根據誤差方向增加最小啟動補償
                    if (error > 0) power += MIN_POWER;
                    else power -= MIN_POWER;

                    power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
                    baseMotor.setPower(power);
                } else {
                    baseMotor.setPower(0);
                }
                lastError = error;
            } else {
                baseMotor.setPower(0);
                lastError = 0;
            }

            // =======================================================
            // 2. 底盤控制
            // =======================================================
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double frontLeftPower = power * cos / max + rx;
            double frontRightPower = power * sin / max - rx;
            double backLeftPower = power * sin / max + rx;
            double backRightPower = power * cos / max - rx;

            if ((power + Math.abs(rx)) > 1) {
                frontLeftPower /= power + Math.abs(rx);
                frontRightPower /= power + Math.abs(rx);
                backLeftPower /= power + Math.abs(rx);
                backRightPower /= power + Math.abs(rx);
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // =======================================================
            // 3. 射擊馬達 PID 控制 (含緩降)
            // =======================================================
            boolean currentRightBumper = gamepad1.right_bumper;
            if (currentRightBumper && !lastRightBumper) isHighSpeedMode = true;
            lastRightBumper = currentRightBumper;

            double currentDistance = 0.0;
            double calculatedRpm = 0.0;

            if (targetValid) {
                double ty = result.getTy();
                currentDistance = calculateDistance(ty);
                if (currentDistance <= DISTANCE_THRESHOLD) {
                    angleServo.setPosition(ANGLE_CLOSE);
                    calculatedRpm = (RPM_SLOPE_CLOSE * currentDistance) + RPM_BASE_CLOSE;
                } else {
                    angleServo.setPosition(ANGLE_FAR);
                    calculatedRpm = (RPM_SLOPE_FAR * currentDistance) + RPM_BASE_FAR;
                }
                calculatedRpm = Math.max(0, Math.min(2800, calculatedRpm));
            } else {
                if (gamepad1.dpad_left && (currentTime - lastInputTime > inputDelay)) {
                    desiredTargetRpm += 50; lastInputTime = currentTime;
                } else if (gamepad1.dpad_right && (currentTime - lastInputTime > inputDelay)) {
                    desiredTargetRpm -= 50; lastInputTime = currentTime;
                }
                calculatedRpm = desiredTargetRpm;
                if(gamepad1.dpad_up) angleServo.setPosition(ANGLE_FAR);
                if(gamepad1.dpad_down) angleServo.setPosition(ANGLE_CLOSE);
            }

            if (isHighSpeedMode) desiredTargetRpm = calculatedRpm;
            else desiredTargetRpm = RPM_IDLE;

            if (desiredTargetRpm >= currentCommandedRpm) {
                currentCommandedRpm = desiredTargetRpm;
            } else {
                currentCommandedRpm -= RPM_RAMP_DOWN_STEP;
                if (currentCommandedRpm < desiredTargetRpm) {
                    currentCommandedRpm = desiredTargetRpm;
                }
            }

            shooterMotorRight.setVelocity(currentCommandedRpm);
            double currentPowerFromRight = shooterMotorRight.getPower();
            shooterMotorLeft.setPower(currentPowerFromRight);

            // =======================================================
            // 4. 手動回退/解卡球功能 (按 A 鍵)
            // =======================================================
            boolean currentButtonA = gamepad1.a;
            if (currentButtonA && !lastButtonA) {
                // 只有在還不是第一步(step>0) 且 沒有在發射時 才能回退
                if (currentFillStep > 0 && fireState == FireState.IDLE) {

                    // 1. 步驟倒退一步
                    currentFillStep--;

                    // 2. 根據退回的步驟，重置Servo位置並清除該洞的記錄
                    if (currentFillStep == 0) {
                        // 退回到 A 洞
                        diskServo.setPosition(FILL_POS_STEP_1);
                        clearBallStatus("A");
                        LED0.off();
                    } else if (currentFillStep == 1) {
                        // 退回到 B 洞
                        diskServo.setPosition(FILL_POS_STEP_2);
                        clearBallStatus("B");
                        LED1.off();
                    } else if (currentFillStep == 2) {
                        // 退回到 C 洞 (原本是FULL狀態)
                        diskServo.setPosition(FILL_POS_STEP_3);
                        clearBallStatus("C");
                        LED2.off();
                    }

                    // 3. 強制讓進球狀態機回到 IDLE，重新開始偵測顏色
                    fillState = FillState.IDLE;
                }
            }
            lastButtonA = currentButtonA;

            // =======================================================
            // 5. 進球/發射邏輯
            // =======================================================
            if (gamepad1.left_bumper && fireState == FireState.IDLE) {
                if (hasBallA || hasBallB || hasBallC) {
                    fireState = FireState.PREPARING;
                    fireTimer = System.currentTimeMillis();
                    controlGates(false);
                }
            }
            runFiringLogic();
            runFillingLogic();

            // Intake 控制
            if(gamepad1.y) {
                intakeMotor.setPower(-1); // 反轉吐球
            } else {
                runIntakeLogic();
            }

            // Telemetry
            if (targetValid) {
                telemetry.addData("Limelight", "Target: RED (ID 24) | Tx: %.2f", result.getTx());
                telemetry.addData("Dist", "%.1f", currentDistance);
            } else {
                telemetry.addData("Limelight", "SEARCHING...");
            }
            telemetry.addData("Fill Step", currentFillStep);
            telemetry.addData("Shooter Target", "%.0f RPM", desiredTargetRpm);
            telemetry.addData("Commanded RPM", "%.0f RPM", currentCommandedRpm);
            telemetry.addData("Actual Vel", "R: %.0f | L: %.0f", shooterMotorRight.getVelocity(), shooterMotorLeft.getVelocity());

            updateTelemetry();
        }
        limelight.stop();
    }

    private double calculateDistance(double ty) {
        double heightDiff = TARGET_HEIGHT - CAMERA_HEIGHT;
        double angleRad = Math.toRadians(MOUNT_ANGLE + ty);
        return heightDiff / Math.tan(angleRad);
    }

    // 邏輯方法
    private void runFillingLogic() { if (fireState != FireState.IDLE) return; if (currentFillStep >= 3) { fillState = FillState.FULL; return; } else if (fillState == FillState.FULL) { fillState = FillState.IDLE; } switch (fillState) { case IDLE: DetectedColor detectedColor = getDualSensorColor(); if (detectedColor != DetectedColor.UNKNOWN) { recordBallColor(detectedColor); fillTimer = System.currentTimeMillis(); fillState = FillState.WAIT_SETTLE; } break; case WAIT_SETTLE: if (System.currentTimeMillis() - fillTimer > TIME_BALL_SETTLE) { moveToNextFillPosition(); fillTimer = System.currentTimeMillis(); fillState = FillState.ROTATING; } break; case ROTATING: if (System.currentTimeMillis() - fillTimer > TIME_DISK_MOVE_INTAKE) { fillState = FillState.IDLE; } break; case FULL: break; } }
    private void runFiringLogic() { switch (fireState) { case IDLE: break; case PREPARING: if (System.currentTimeMillis() - fireTimer > TIME_SHOOTER_SPIN) fireState = FireState.DECIDING; break; case DECIDING: if (hasBallC) { targetFirePos = FIRE_POS_HOLE_C; currentTargetHole = "C"; switchToAiming(); LED2.off(); } else if (hasBallB) { targetFirePos = FIRE_POS_HOLE_B; currentTargetHole = "B"; switchToAiming(); LED1.off(); } else if (hasBallA) { targetFirePos = FIRE_POS_HOLE_A; currentTargetHole = "A"; switchToAiming(); LED0.off(); } else { fireTimer = System.currentTimeMillis(); fireState = FireState.RESETTING; diskServo.setPosition(FILL_POS_STEP_1); } break; case AIMING: if (System.currentTimeMillis() - fireTimer > TIME_DISK_MOVE_SHOOTING) { kickerServo.setPosition(KICKER_EXTEND); fireTimer = System.currentTimeMillis(); fireState = FireState.KICKING; } break; case KICKING: if (System.currentTimeMillis() - fireTimer > TIME_KICK_OUT) { kickerServo.setPosition(KICKER_REST); clearBallStatus(currentTargetHole); fireTimer = System.currentTimeMillis(); fireState = FireState.RETRACTING; } break; case RETRACTING: if (System.currentTimeMillis() - fireTimer > TIME_KICK_RETRACT) fireState = FireState.DECIDING; break; case RESETTING: if (System.currentTimeMillis() - fireTimer > 600) { controlGates(true); currentFillStep = 0; fireState = FireState.IDLE; isHighSpeedMode = false; } break; } }
    private void switchToAiming() { diskServo.setPosition(targetFirePos); fireTimer = System.currentTimeMillis(); fireState = FireState.AIMING; }
    private void runIntakeLogic() { if (currentFillStep < 3 && fireState == FireState.IDLE) { intakeMotor.setPower(INTAKE_POWER); } else { intakeMotor.setPower(0.0); } }
    private void recordBallColor(DetectedColor color) { switch (currentFillStep) { case 0: colorHoleA = color.toString(); hasBallA = true; LED0.on(); break; case 1: colorHoleB = color.toString(); hasBallB = true; LED1.on(); break; case 2: colorHoleC = color.toString(); hasBallC = true; LED2.on(); break; } }
    private void moveToNextFillPosition() { if (currentFillStep == 0) { diskServo.setPosition(FILL_POS_STEP_2); currentFillStep = 1; } else if (currentFillStep == 1) { diskServo.setPosition(FILL_POS_STEP_3); currentFillStep = 2; } else if (currentFillStep == 2) { currentFillStep = 3; } }

    private void clearBallStatus(String hole) {
        if (hole.equals("A")) { hasBallA = false; colorHoleA = "EMPTY"; }
        if (hole.equals("B")) { hasBallB = false; colorHoleB = "EMPTY"; }
        if (hole.equals("C")) { hasBallC = false; colorHoleC = "EMPTY"; }
    }

    private void controlGates(boolean isOpen) { if (isOpen) { gateServoL.setPosition(GATE_L_OPEN); gateServoR.setPosition(GATE_R_OPEN); } else { gateServoL.setPosition(GATE_CLOSED); gateServoR.setPosition(GATE_CLOSED); } }
    private DetectedColor getDualSensorColor() { DetectedColor c1 = getDetectedColor(colorSensor1); DetectedColor c2 = getDetectedColor(colorSensor2); if (c1 != DetectedColor.UNKNOWN) return c1; if (c2 != DetectedColor.UNKNOWN) return c2; return DetectedColor.UNKNOWN; }
    public DetectedColor getDetectedColor(NormalizedColorSensor sensor) { NormalizedRGBA color = sensor.getNormalizedColors(); if (color.alpha < MIN_DETECT_BRIGHTNESS) return DetectedColor.UNKNOWN; if (color.blue > color.green && color.blue > color.red) { if (color.blue > (color.green * PURPLE_RATIO_LIMIT)) return DetectedColor.PURPLE; } if (color.green > color.red) { if (color.green >= color.blue || (color.green > color.blue * 0.85f)) return DetectedColor.GREEN; } return DetectedColor.UNKNOWN; }

    private void initHardware() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        if (colorSensor1 instanceof com.qualcomm.robotcore.hardware.SwitchableLight) ((com.qualcomm.robotcore.hardware.SwitchableLight)colorSensor1).enableLight(true);
        if (colorSensor2 instanceof com.qualcomm.robotcore.hardware.SwitchableLight) ((com.qualcomm.robotcore.hardware.SwitchableLight)colorSensor2).enableLight(true);
        colorSensor1.setGain(SENSOR_GAIN); colorSensor2.setGain(SENSOR_GAIN);
        kickerServo = hardwareMap.get(Servo.class, "servo1");
        diskServo = hardwareMap.get(Servo.class, "servo2");
        gateServoL = hardwareMap.get(Servo.class, "servo4");
        gateServoR = hardwareMap.get(Servo.class, "servo5");
        angleServo = hardwareMap.get(Servo.class, "servo3");
        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");

        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "motor5");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "motor7");
        shooterMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
        shooterMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
        baseMotor = hardwareMap.get(DcMotor.class, "motor6");
        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kickerServo.scaleRange(0.0, 0.5);
        gateServoL.setDirection(Servo.Direction.REVERSE);
        gateServoR.setDirection(Servo.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kickerServo.setPosition(KICKER_REST);
        diskServo.setPosition(FILL_POS_STEP_1);
        controlGates(true);
        intakeMotor.setPower(0);
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);
        baseMotor.setPower(0);
        LED0 = hardwareMap.get(LED.class, "LED0");
        LED1 = hardwareMap.get(LED.class, "LED1");
        LED2 = hardwareMap.get(LED.class, "LED2");
        LED0.off(); LED1.off(); LED2.off();
    }

    private void updateTelemetry() {
        telemetry.addLine("=== RED ALLIANCE SYSTEM ===");
        telemetry.addData("Fill State", fillState);
        telemetry.addData("Fire State", fireState);
        telemetry.addData("Target", currentTargetHole);
        if (fireState != FireState.IDLE) telemetry.addData("Action", "FIRING"); else telemetry.addData("Action", "Intake / Idle");
        telemetry.addLine("\n=== BALLS ===");
        telemetry.addData("A", "[%s] %s (LED0)", colorHoleA, hasBallA?"●":"○");
        telemetry.addData("B", "[%s] %s (LED1)", colorHoleB, hasBallB?"●":"○");
        telemetry.addData("C", "[%s] %s (LED2)", colorHoleC, hasBallC?"●":"○");
        telemetry.update();
    }
}