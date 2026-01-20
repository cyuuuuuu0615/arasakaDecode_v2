package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop_v6")
public class teleop_v6 extends LinearOpMode {

    public static double handlerange(double x, double a, double b) {
        if (x > a) {
            return a;
        } else if (x < b) {
            return b;
        } else {
            return x;
        }
    }

    // === 硬件變量 ===
    NormalizedColorSensor colorSensor1, colorSensor2;
    Servo kickerServo, diskServo, gateServoL, gateServoR, angleServo; // angleServo moved here
    DcMotor intakeMotor, shooterMotor, baseMotor;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // === 參數設定 ===
    private static final double FILL_POS_STEP_1 = 0.0;     // Hole A
    private static final double FILL_POS_STEP_2 = 0.3529;  // Hole B
    private static final double FILL_POS_STEP_3 = 0.7137;  // Hole C

    private static final double FIRE_POS_HOLE_B = 0.0471;
    private static final double FIRE_POS_HOLE_C = 0.4314;
    private static final double FIRE_POS_HOLE_A = 0.8196;

    private static final double KICKER_REST = 0.0;
    private static final double KICKER_EXTEND = 0.8;

    private static final int TIME_BALL_SETTLE = 300;
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
    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }

    // === 新增：發射預設模式 ===
    private enum ShootingPreset { MANUAL, PRESET_A, PRESET_B, PRESET_X }
    private ShootingPreset currentPreset = ShootingPreset.MANUAL;

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

    // 手動變量
    private double motorPowerVariable = 0;

    @Override
    public void runOpMode() {
        initHardware();

        // Base Motor 設定
        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Angle Servo 設定
        angleServo.setDirection(Servo.Direction.REVERSE);
        // angleServo.scaleRange(0.0, 0.16); // 保留您的註解

        telemetry.addData("Status", "Initialized with Presets");
        telemetry.update();

        long lastInputTime = 0;
        long inputDelay = 200;

        // Base Motor Limits
        int b_upper_limit = 80;
        int b_lower_limit = -231;

        waitForStart();

        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();

            // =========================================================
            // 1. Gamepad 2 自動預設 (Auto Presets)
            // =========================================================

            // Preset 1: Gamepad 2 B (對應描述1)
            if (gamepad2.b) {
                currentPreset = ShootingPreset.PRESET_B;
                activatePreset(-52, 0.2); // Base: -52, Angle: 0.2
            }
            // Preset 2: Gamepad 2 A (對應描述2)
            else if (gamepad2.a) {
                currentPreset = ShootingPreset.PRESET_A;
                activatePreset(-25, 0.2); // Base: -25, Angle: 0.2
            }
            // Preset 3: Gamepad 2 X (對應描述3，原描述為B，改為X以區分)
            else if (gamepad2.y) {
                currentPreset = ShootingPreset.PRESET_X;
                activatePreset(-51, 0.1); // Base: -51, Angle: 0.1
            }

            // =========================================================
            // 2. Gamepad 1 手動控制與優先級 (Manual Override)
            // =========================================================

            // 手動調整力度 (會切換回 Manual)
            if (gamepad2.dpad_left && (currentTime - lastInputTime > inputDelay)) {
                motorPowerVariable += 0.05;
                lastInputTime = currentTime;
                currentPreset = ShootingPreset.MANUAL; // 切換回手動
            } else if (gamepad2.dpad_right && (currentTime - lastInputTime > inputDelay)) {
                motorPowerVariable -= 0.05;
                lastInputTime = currentTime;
                currentPreset = ShootingPreset.MANUAL; // 切換回手動
            }

            // 安全限制
            if (motorPowerVariable > 1.0) motorPowerVariable = 1.0;
            if (motorPowerVariable < -1.0) motorPowerVariable = -1.0;

            // 手動調整 Base Motor 高度 (會切換回 Manual)
            double baseManualPower = (-gamepad2.left_trigger + gamepad2.right_trigger);
            if (Math.abs(baseManualPower) > 0.05) {
                currentPreset = ShootingPreset.MANUAL; // 只要動了 trigger 就切回手動
                baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if (baseMotor.getCurrentPosition() <= b_upper_limit && baseMotor.getCurrentPosition() >= b_lower_limit) {
                    baseMotor.setPower(handlerange(baseManualPower, 1, -1));
                } else if (baseMotor.getCurrentPosition() >= b_upper_limit) {
                    baseMotor.setPower(handlerange(baseManualPower, 0, -1));
                } else if (baseMotor.getCurrentPosition() <= b_lower_limit) {
                    baseMotor.setPower(handlerange(baseManualPower, 1, 0));
                }
            } else if (currentPreset == ShootingPreset.MANUAL) {
                // 如果在手動模式且沒按 Trigger，保持不動
                baseMotor.setPower(0);
            }

            // 手動調整 Angle Servo
            if (gamepad2.dpad_up) {
                angleServo.setPosition(0.2);
                currentPreset = ShootingPreset.MANUAL;
            }
            if (gamepad2.dpad_down) {
                angleServo.setPosition(0);
                currentPreset = ShootingPreset.MANUAL;
            }
            if (gamepad2.a) {
                angleServo.setPosition(0.1);
                currentPreset = ShootingPreset.MANUAL;
            }

            // =========================================================
            // 3. 底盤控制 (Mecanum Drive)
            // =========================================================
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

            // =========================================================
            // 4. 手動發射觸發 (如果是自動預設，會自動觸發，不需要這裡)
            // =========================================================
            if (gamepad2.left_bumper && fireState == FireState.IDLE) {
                if (hasBallA || hasBallB || hasBallC) {
                    fireState = FireState.PREPARING;
                    fireTimer = System.currentTimeMillis();
                    controlGates(false);
                }
            }

            // === 執行邏輯 ===
            // 這裡不再直接設定 shooterMotor.setPower，而是交給 runFiringLogic 處理
            runFiringLogic();
            runFillingLogic();
            runIntakeLogic();

            updateTelemetry();
        }
    }

    // === 輔助方法：激活預設 ===
    private void activatePreset(int targetPos, double anglePos) {
        // 設定 Angle Servo
        angleServo.setPosition(anglePos);

        // 設定 Base Motor 到指定位置
        baseMotor.setTargetPosition(targetPos);
        baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseMotor.setPower(0.8); // 給予足夠的速度移動到位置

        // 自動觸發發射
        if (fireState == FireState.IDLE && (hasBallA || hasBallB || hasBallC)) {
            fireState = FireState.PREPARING;
            fireTimer = System.currentTimeMillis();
            controlGates(false);
        }
    }

    // === 裝球邏輯 (不變) ===
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

    // === 發射邏輯 (修改：加入力度控制) ===
    private void runFiringLogic() {
        // 根據當前狀態設定馬達力度
        updateShooterPower();

        switch (fireState) {
            case IDLE:
                // IDLE 時如果不轉動，可以省電或保持怠速，這裡設為0或根據需求
                if (currentPreset == ShootingPreset.MANUAL) shooterMotor.setPower(motorPowerVariable);
                else shooterMotor.setPower(0);
                break;

            case PREPARING:
                if (System.currentTimeMillis() - fireTimer > TIME_SHOOTER_SPIN) fireState = FireState.DECIDING;
                break;

            case DECIDING:
                // 發射順序：C -> B -> A
                if (hasBallC) {
                    targetFirePos = FIRE_POS_HOLE_C;
                    currentTargetHole = "C";
                    switchToAiming();
                }
                else if (hasBallB) {
                    targetFirePos = FIRE_POS_HOLE_B;
                    currentTargetHole = "B";
                    switchToAiming();
                }
                else if (hasBallA) {
                    targetFirePos = FIRE_POS_HOLE_A;
                    currentTargetHole = "A";
                    switchToAiming();
                }
                else {
                    // 完成
                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.RESETTING;
                    diskServo.setPosition(FILL_POS_STEP_1);
                    // 結束後可以切回手動或保持
                    currentPreset = ShootingPreset.MANUAL;
                    baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    // === 新增：根據預設和當前球設定力度 ===
    private void updateShooterPower() {
        if (fireState == FireState.IDLE) return; // IDLE 由外部控制

        double power = 0;

        if (currentPreset == ShootingPreset.MANUAL) {
            power = motorPowerVariable;
        } else {
            // 決定當前準備發射的是哪一顆 (順序 C -> B -> A)
            // 這裡邏輯：如果是 PREPARING 或 DECIDING，我們預判下一顆是誰
            // 如果是 AIMING/KICKING，currentTargetHole 已經設定好了

            String target = currentTargetHole;
            // 如果還沒設定目標 (PREPARING)，預判順序
            if (fireState == FireState.PREPARING || fireState == FireState.DECIDING) {
                if (hasBallC) target = "C";
                else if (hasBallB) target = "B";
                else if (hasBallA) target = "A";
            }

            switch (currentPreset) {
                case PRESET_B: // Gamepad 2 B
                    if (target.equals("C")) power = 0.5;        // 第一顆
                    else if (target.equals("B")) power = 0.55;  // 第二顆
                    else if (target.equals("A")) power = 0.55;  // 第三顆
                    break;

                case PRESET_A: // Gamepad 2 A
                    if (target.equals("C")) power = 0.7;        // 第一顆
                    else if (target.equals("B")) power = 0.75;  // 第二顆
                    else if (target.equals("A")) power = 0.8;   // 第三顆
                    break;

                case PRESET_X: // Gamepad 2 X (Base -51)
                    if (target.equals("C")) power = 0.45;       // 第一顆
                    else power = 0.5;                           // 第二、三顆
                    break;
            }
        }

        shooterMotor.setPower(power);
        telemetry.addData("Auto Power", power);
    }

    private void switchToAiming() {
        diskServo.setPosition(targetFirePos);
        fireTimer = System.currentTimeMillis();
        fireState = FireState.AIMING;
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
        angleServo = hardwareMap.get(Servo.class, "servo3"); // Added here

        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
        shooterMotor = hardwareMap.get(DcMotor.class, "motor5");
        baseMotor = hardwareMap.get(DcMotor.class, "motor6"); // Added here

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
        telemetry.addData("Preset Mode", currentPreset);
        telemetry.addData("Target", currentTargetHole);

        if (fireState != FireState.IDLE) telemetry.addData("Action", "FIRING (Priority: C->B->A)");
        else telemetry.addData("Action", "Intake / Idle");

        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.addData("Base Pos", baseMotor.getCurrentPosition());

        telemetry.addLine("\n=== BALLS ===");
        telemetry.addData("A", "[%s] %s", colorHoleA, hasBallA?"●":"○");
        telemetry.addData("B", "[%s] %s", colorHoleB, hasBallB?"●":"○");
        telemetry.addData("C", "[%s] %s", colorHoleC, hasBallC?"●":"○");
        telemetry.update();
    }
}