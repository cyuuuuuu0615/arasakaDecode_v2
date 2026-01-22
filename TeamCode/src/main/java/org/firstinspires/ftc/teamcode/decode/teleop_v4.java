package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "A teleop_one")
public class teleop_v4 extends LinearOpMode {

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
    DcMotor intakeMotor, shooterMotorLeft, shooterMotorRight, baseMotor;
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
        initHardware();
        DcMotor baseMotor = hardwareMap.get(DcMotor.class, "motor6");
        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Servo angleServo = hardwareMap.get(Servo.class,"servo3");
        angleServo.setDirection(Servo.Direction.REVERSE);
//        angleServo.scaleRange(0.0, 0.16);




        telemetry.addData("Status", "Sequence Updated");
        telemetry.addData("Priority", "C -> B -> A"); // 顯示優先級
        telemetry.addData("Intake Power", INTAKE_POWER);
        telemetry.update();

        double motorPowerVariable = 0;

        // --- 計時器變數設定 ---
        // 用來記錄上一次按下按鈕的時間
        long lastInputTime = 0;
        // 設定冷卻時間 (毫秒)，這裡設為 200ms (0.2秒)
        long inputDelay = 200;

        waitForStart();
        int blp = baseMotor.getCurrentPosition();
        int b_upper_limit = 473;
        int b_lower_limit = 0;
        float a_upper_limit = 0.1314F;
        int a_lower_limit = 0;
        motorPowerVariable = 0;


//        angleServo.setPosition(0);
        while (opModeIsActive()) {

            // 獲取當前系統時間
            long currentTime = System.currentTimeMillis();

            // 1. 檢測手把輸入 (加入時間判斷)

            // 邏輯：如果 (按下了左鍵) 且 (現在時間 - 上次按下的時間 > 延遲時間)
            if (gamepad1.dpad_left && (currentTime - lastInputTime > inputDelay)) {

                motorPowerVariable = motorPowerVariable + 0.05;

                // 更新 "上次按下時間" 為 "現在時間"
                lastInputTime = currentTime;
            }
            // 邏輯：如果 (按下了右鍵) 且 (冷卻時間已到)
            else if (gamepad1.dpad_right && (currentTime - lastInputTime > inputDelay)) {

                motorPowerVariable = motorPowerVariable - 0.05;

                // 更新計時器
                lastInputTime = currentTime;
            }

            // --- 安全限制 (可選) ---
            // 為了防止數值超過 1.0 或 低於 -1.0，建議加上這段 (Block 裡沒有，但 Java 建議加)
            if (motorPowerVariable > 1.0) motorPowerVariable = 1.0;
            if (motorPowerVariable < -1.0) motorPowerVariable = -1.0;

            // 2. 設定馬達功率
            shooterMotorLeft.setPower(motorPowerVariable);
            shooterMotorRight.setPower(motorPowerVariable);



//            baseMotor.setPower((gamepad1.left_trigger - gamepad1.right_trigger)*0.5);
            if (baseMotor.getCurrentPosition() <= b_upper_limit && baseMotor.getCurrentPosition() >= b_lower_limit) {
                baseMotor.setPower(handlerange((-gamepad1.left_trigger + gamepad1.right_trigger)*0.5,1,-1));
            } else if (baseMotor.getCurrentPosition() >= b_upper_limit) {
                baseMotor.setPower(handlerange((-gamepad1.left_trigger + gamepad1.right_trigger)*0.5, 0, -1));
            } else if (baseMotor.getCurrentPosition() <= b_lower_limit) {
                baseMotor.setPower(handlerange((-gamepad1.left_trigger + gamepad1.right_trigger)*0.5, 1, 0));
            }
            telemetry.addData("motor6 current position",baseMotor.getCurrentPosition());
            telemetry.addData("Actual Left Motor Power", shooterMotorLeft.getPower());
            telemetry.addData("Actual Right Motor Power", shooterMotorRight.getPower());
            telemetry.addData("servo3 position",angleServo.getPosition());




            if(gamepad1.dpad_up){
                angleServo.setPosition(0.2);
            }
            if(gamepad1.dpad_down){
                angleServo.setPosition(0);
            }
            if(gamepad1.a){
                angleServo.setPosition(0.1);
            }



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
//                    shooterMotor.setPower(0.8);
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
                if (System.currentTimeMillis() - fillTimer > TIME_DISK_MOVE_INTAKE) {
                    fillState = FillState.IDLE;
                }
                break;

            case FULL: break;
        }
    }

    // === 發射邏輯 (順序：C -> B -> A) ===
    private void runFiringLogic() {
        switch (fireState) {
            case IDLE: break;
            case PREPARING:
                if (System.currentTimeMillis() - fireTimer > TIME_SHOOTER_SPIN) fireState = FireState.DECIDING;
                break;

            case DECIDING:
                // [關鍵修改] 強制發射順序：C -> B -> A

                // 1. 優先檢查 C (0.4314)
                if (hasBallC) {
                    targetFirePos = FIRE_POS_HOLE_C;
                    currentTargetHole = "C";
                    switchToAiming();
                }
                // 2. 其次檢查 B (0.0471)
                else if (hasBallB) {
                    targetFirePos = FIRE_POS_HOLE_B;
                    currentTargetHole = "B";
                    switchToAiming();
                }
                // 3. 最後檢查 A (0.8196)
                else if (hasBallA) {
                    targetFirePos = FIRE_POS_HOLE_A;
                    currentTargetHole = "A";
                    switchToAiming();
                }
                else {
                    // 全部發射完畢，復位
                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.RESETTING;
//                    shooterMotor.setPower(0.0);
                    diskServo.setPosition(FILL_POS_STEP_1); // 回到 0.0 準備 Intake
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
                    currentFillStep = 0; // 重置計數
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



        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kickerServo.scaleRange(0.0, 0.5);
        gateServoL.setDirection(Servo.Direction.REVERSE);
        gateServoR.setDirection(Servo.Direction.FORWARD);
        shooterMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        kickerServo.setPosition(KICKER_REST);
        diskServo.setPosition(FILL_POS_STEP_1);
        controlGates(true);
        intakeMotor.setPower(0);
        shooterMotorLeft.setPower(0);
        shooterMotorRight.setPower(0);
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