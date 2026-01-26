package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; // Added for PID control
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients; // Added for PID control
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop_v3_limelight+PID")
public class teleop_v3 extends LinearOpMode {

    // === 來自 Program 1 的 PID Tuning 變數 ===
    public double highVelocity = 1500;
    public double lowVelocity = 900;
    double curTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    // 用於按鍵單次觸發判斷 (模擬 wasPressed)
    boolean lastY = false, lastX = false, lastB = false;
    boolean lastDpadLeft = false, lastDpadRight = false, lastDpadUp = false, lastDpadDown = false;

    // === Limelight 與 PD 控制參數 (來自 Motor_v4) ===
    private Limelight3A limelight;
    private final double TARGET_TX = 8.0;
    private double KP = 0.018;
    private double KD = 0.025;
    private final double MAX_POWER = 0.45;
    private final double MIN_POWER = 0.06;
    private final double DEADBAND = 1.0;
    private double lastError = 0;

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
    DcMotor intakeMotor, baseMotor;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // 將 Shooter 改為 DcMotorEx 以支援 PIDF 控制
    DcMotorEx shooterMotorLeft, shooterMotorRight;

    // === 參數設定 ===
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

    @Override
    public void runOpMode() {
        initHardware();

        Servo angleServo = hardwareMap.get(Servo.class,"servo3");
        angleServo.setDirection(Servo.Direction.REVERSE);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Limelight Tracking & PID Initialized");
        telemetry.update();
        curTargetVelocity = 100;

        waitForStart();

        while (opModeIsActive()) {

            // =======================================================
            // === Program 1 邏輯整合: PID Tuning 與 Velocity 控制 ===
            // =======================================================

            // 按鍵狀態檢測 (模擬 WasPressed)
            boolean currY = gamepad1.y;
            boolean currX = gamepad1.x;
            boolean currB = gamepad1.b;
            boolean currDpadLeft = gamepad1.dpad_left;
            boolean currDpadRight = gamepad1.dpad_right;
            boolean currDpadUp = gamepad1.dpad_up;
            boolean currDpadDown = gamepad1.dpad_down;

            // 切換速度目標 (Y)
//            if(currY && !lastY){
//                if(curTargetVelocity == highVelocity){
//                    curTargetVelocity = lowVelocity;
//                }else{
//                    curTargetVelocity = highVelocity;
//                }
//            }

            if(currX && !lastX){
                curTargetVelocity += 100;
            }
            if(currY && !lastY){
                curTargetVelocity -= 100;
            }

            // 切換微調步長 (B)
            if(currB && !lastB){
                stepIndex = (stepIndex+1) % stepSizes.length;
            }

            // 調整 F (D-pad Left/Right)
            if(currDpadLeft && !lastDpadLeft){
                F -= stepSizes[stepIndex];
            }
            if(currDpadRight && !lastDpadRight){
                F += stepSizes[stepIndex];
            }

            // 調整 P (D-pad Up/Down)
            if(currDpadUp && !lastDpadUp){
                P += stepSizes[stepIndex];
            }
            if(currDpadDown && !lastDpadDown){
                P -= stepSizes[stepIndex];
            }

            // 更新按鍵狀態
            lastY = currY;
            lastB = currB;
            lastDpadLeft = currDpadLeft;
            lastDpadRight = currDpadRight;
            lastDpadUp = currDpadUp;
            lastDpadDown = currDpadDown;

            // 設定新的 PIDF 係數 (只對 Right 設定，因為 Left 沒有 Encoder 回授)
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
            shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            // 設定速度 (主動輪 Right)
            shooterMotorRight.setVelocity(curTargetVelocity);

            // === 關鍵整合: 獲取 Right 的實際動力並應用到 Left ===
            // 這樣 Left 會跟隨 PID 計算出的 Power，但不需要自己的 Encoder
            double appliedPower = shooterMotorRight.getPower();
            shooterMotorLeft.setPower(appliedPower);

            // =======================================================
            // === Program 2 邏輯: Limelight 自動追蹤 ===
            // =======================================================

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double error = tx - TARGET_TX;
                double errorChange = error - lastError;
                double dTerm = errorChange * KD;

                if (Math.abs(error) > DEADBAND) {
                    double power = (error * KP) + dTerm;
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
            // === Program 2 邏輯: 伺服機與底盤控制 ===
            // =======================================================

            if(gamepad1.dpad_up){
                angleServo.setPosition(0.2);
            }
            if(gamepad1.dpad_down){
                angleServo.setPosition(0);
            }
            if(gamepad1.a){
                angleServo.setPosition(0.1);
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
            if(gamepad1.y){ // 注意：這裡 Y 鍵同時也會觸發上面的速度切換
                intakeMotor.setPower(-1);
            }else{
                runIntakeLogic();
            }

            updateTelemetry();
        }

        limelight.stop();
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
                // 在此狀態下，PID 控制已經在主循環中讓馬達旋轉，這裡只需等待
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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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

        // === Shooter Motor 初始化 (DcMotorEx) ===
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "motor5");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "motor7");

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

        // === Shooter Motor 方向與模式設定 ===
        // 假設 Left 需要反轉以配合 Right (物理上對置)
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // === PIDF 初始化設定 (只針對 Right) ===
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Left 設為不使用 Encoder，單純接收 Power 指令
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kickerServo.setPosition(KICKER_REST);
        diskServo.setPosition(FILL_POS_STEP_1);
        controlGates(true);
        intakeMotor.setPower(0);
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);
        baseMotor.setPower(0);
    }

    private void updateTelemetry() {
        telemetry.addLine("=== PID TUNING ===");
        double curVelocity = shooterMotorRight.getVelocity();
        double error = curTargetVelocity - curVelocity;
        telemetry.addData("Target V", curTargetVelocity);
        telemetry.addData("Current V", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("P (D-Pad U/D)", "%.4f", P);
        telemetry.addData("F (D-Pad L/R)", "%.4f", F);
        telemetry.addData("Step (B Btn)", "%.4f", stepSizes[stepIndex]);
        telemetry.addLine("------------------------------------");

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