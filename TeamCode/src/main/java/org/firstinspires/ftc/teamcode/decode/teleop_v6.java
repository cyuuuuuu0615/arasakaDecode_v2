package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "A Teleop_v6_AutoAim")
public class teleop_v6 extends LinearOpMode {

    // === Limelight 與 PD 控制參數 (來自 Motor_v4) ===
    private Limelight3A limelight;

    // --- 關鍵參數調整 ---
    private final double TARGET_TX = 8.0;

    // P 決定追蹤力量
    private double KP = 0.016;

    // D 決定煞車力量
    private double KD = 0.073;

    // 限制物理速度
    private final double MAX_POWER = 0.45;

    // 最小啟動動力
    private final double MIN_POWER = 0.06;

    private final double DEADBAND = 1.0;
    private double lastError = 0;
    // ============================================

    public static double handlerange(double x,double a,double b){
        if(x>a){
            return a;
        }else if(x<b){
            return b;
        }else{
            return x;
        }
    }

    // === 硬件變量 ===
    NormalizedColorSensor colorSensor1, colorSensor2;
    Servo kickerServo, diskServo, gateServoL, gateServoR;
    DcMotor intakeMotor, shooterMotorLeft, shooterMotorRight, baseMotor; // 注意: baseMotor 即 motor6
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
    private static final int TIME_BALL_SETTLE = 150;
    private static final int TIME_DISK_MOVE_INTAKE = 350;
    private static final int TIME_DISK_MOVE_SHOOTING = 500;
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

    // Intake Power
    private static final double INTAKE_POWER = 0.6;

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
        // 硬體初始化整合到 initHardware 方法中
        initHardware();

        Servo angleServo = hardwareMap.get(Servo.class,"servo3");
        angleServo.setDirection(Servo.Direction.FORWARD);

        // 啟動 Limelight
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Limelight Tracking Initialized");
        telemetry.addData("Priority", "C -> B -> A");
        telemetry.update();

        double motorPowerVariable = 0;

        // --- 計時器變數設定 ---
        long lastInputTime = 0;
        long inputDelay = 200;

        waitForStart();

        // 這些變數在自動追蹤模式下可能暫時用不到，但保留變數定義
        int blp = baseMotor.getCurrentPosition();
        motorPowerVariable = 0;

        while (opModeIsActive()) {

            long currentTime = System.currentTimeMillis();

            // 1. 檢測手把輸入 (調整 Shooter 速度)
            if (gamepad1.dpad_left && (currentTime - lastInputTime > inputDelay)) {
                motorPowerVariable = motorPowerVariable + 0.05;
                lastInputTime = currentTime;
            }
            else if (gamepad1.dpad_right && (currentTime - lastInputTime > inputDelay)) {
                motorPowerVariable = motorPowerVariable - 0.05;
                lastInputTime = currentTime;
            }

            // 安全限制
            if (motorPowerVariable > 1.0) motorPowerVariable = 1.0;
            if (motorPowerVariable < -1.0) motorPowerVariable = -1.0;

            shooterMotorLeft.setPower(motorPowerVariable);
            shooterMotorRight.setPower(motorPowerVariable);

            // =======================================================
            // === Motor6 (baseMotor) 自動追蹤邏輯 (取代原本的手動控制) ===
            // =======================================================

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double error = tx - TARGET_TX;

                // 計算 D 項
                double errorChange = error - lastError;
                double dTerm = errorChange * KD;

                if (Math.abs(error) > DEADBAND) {
                    // PD 控制公式
                    double power = (error * KP) + dTerm;

                    // 加上最小動力補償
                    if (error > 0) power += MIN_POWER;
                    else power -= MIN_POWER;

                    // 最終限制
                    power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
                    baseMotor.setPower(power);
                } else {
                    baseMotor.setPower(0);
                }

                lastError = error; // 更新上一次誤差
                telemetry.addData("Limelight", "Tracking | tx: %.2f", tx);
            } else {
                // 失去目標時停止
                baseMotor.setPower(0);
                lastError = 0;
                telemetry.addData("Limelight", "LOST - Searching");
            }

            // =======================================================

            telemetry.addData("motor6 Power", baseMotor.getPower());
            telemetry.addData("Actual Left Motor Power", shooterMotorLeft.getPower());
            telemetry.addData("Actual Right Motor Power", shooterMotorRight.getPower());
            telemetry.addData("servo3 position", angleServo.getPosition());

            // 角度伺服機控制
            if(gamepad1.dpad_up){
                angleServo.setPosition(0.08);
            }
            if(gamepad1.dpad_down){
                angleServo.setPosition(0.19);
            }


            // 底盤移動邏輯 (Mecanum)
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
                }
            }

            // === 執行邏輯 ===
            runFiringLogic();
            runFillingLogic();
            if(gamepad1.y){
                intakeMotor.setPower(-1);
            }else{
                runIntakeLogic();
            }

            updateTelemetry();
        }

        limelight.stop(); // 結束時關閉 Limelight
    }

    // === 裝球邏輯 (保持不變) ===
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
                if (System.currentTimeMillis() - fillTimer > TIME_DISK_MOVE_INTAKE) {
                    fillState = FillState.IDLE;
                }
                break;

            case FULL: break;
        }
    }

    // === 發射邏輯 (保持不變) ===
    private void runFiringLogic() {
        switch (fireState) {
            case IDLE: break;
            case PREPARING:
                if (System.currentTimeMillis() - fireTimer > TIME_SHOOTER_SPIN) fireState = FireState.DECIDING;
                break;

            case DECIDING:
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
                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.RESETTING;
                    diskServo.setPosition(FILL_POS_STEP_1);
                }
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
                if (System.currentTimeMillis() - fireTimer > 600) {
                    controlGates(true);
                    currentFillStep = 0;
                    fireState = FireState.IDLE;
                }
                break;
        }
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
        // Limelight 初始化
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // 傳感器與其他馬達初始化
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
        shooterMotorLeft = hardwareMap.get(DcMotor.class, "motor5");
        shooterMotorRight = hardwareMap.get(DcMotor.class, "motor7");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");

        // === Motor6 / BaseMotor 初始化調整 ===
        baseMotor = hardwareMap.get(DcMotor.class, "motor6");
        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // 原本使用 RUN_USING_ENCODER，但 PD 控制通常使用 RUN_WITHOUT_ENCODER 以獲得即時響應
        baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 保持鎖死

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kickerServo.scaleRange(0.0, 0.5);
        gateServoL.setDirection(Servo.Direction.REVERSE);
        gateServoR.setDirection(Servo.Direction.FORWARD);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        kickerServo.setPosition(KICKER_REST);
        diskServo.setPosition(FILL_POS_STEP_1);
        controlGates(true);
        intakeMotor.setPower(0);
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);
        baseMotor.setPower(0);
    }

    private void updateTelemetry() {
        telemetry.addLine("=== SYSTEM STATUS ===");
        telemetry.addData("Fill State", fillState);
        telemetry.addData("Fire State", fireState);
        telemetry.addData("Target", currentTargetHole);

        if (fireState != FireState.IDLE) telemetry.addData("Action", "FIRING (Priority: C->B->A)");
        else telemetry.addData("Action", "Intake / Idle");

        telemetry.addLine("\n=== BALLS ===");
        telemetry.addData("A", "[%s] %s", colorHoleA, hasBallA?"●":"○");
        telemetry.addData("B", "[%s] %s", colorHoleB, hasBallB?"●":"○");
        telemetry.addData("C", "[%s] %s", colorHoleC, hasBallC?"●":"○");
        telemetry.update();
    }
}