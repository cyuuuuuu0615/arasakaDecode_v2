//package org.firstinspires.ftc.teamcode.decode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@TeleOp(name = "teleop_v6")
//public class teleop_v6 extends LinearOpMode {
//
//    public static double handlerange(double x, double a, double b) {
//        if (x > a) {
//            return a;
//        } else if (x < b) {
//            return b;
//        } else {
//            return x;
//        }
//    }
//
//    // === 硬件變量 ===
//    NormalizedColorSensor colorSensor1, colorSensor2;
//    Servo kickerServo, diskServo, gateServoL, gateServoR;
//    DcMotor intakeMotor, shooterMotor, baseMotor;
//    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
//
//    // === 參數設定 ===
//    // 裝球位置 (Filling)
//    private static final double FILL_POS_STEP_1 = 0.0;     // Hole A (Step 0)
//    private static final double FILL_POS_STEP_2 = 0.3529;  // Hole B (Step 1)
//    private static final double FILL_POS_STEP_3 = 0.7137;  // Hole C (Step 2)
//
//    // 發射位置 (Firing)
//    private static final double FIRE_POS_HOLE_B = 0.0471;
//    private static final double FIRE_POS_HOLE_C = 0.4314;
//    private static final double FIRE_POS_HOLE_A = 0.8196;
//
//    // Kicker
//    private static final double KICKER_REST = 0.0;
//    private static final double KICKER_EXTEND = 0.8;
//
//    // 時間參數 (ms)
//    private static final int TIME_BALL_SETTLE = 100;
//    private static final int TIME_DISK_MOVE = 300;
//    private static final int TIME_SHOOTER_SPIN = 1000;
//    private static final int TIME_KICK_OUT = 300;
//    private static final int TIME_KICK_RETRACT = 250;
//
//    // 閘門
//    private static final double GATE_CLOSED = 0.0;
//    private static final double GATE_L_OPEN = 0.6667;
//    private static final double GATE_R_OPEN = 0.6902;
//
//    // 傳感器與馬達
//    private static final float SENSOR_GAIN = 25.0f;
//    private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
//    private static final float PURPLE_RATIO_LIMIT = 1.2f;
//
//    // Intake Power
//    private static final double INTAKE_POWER = 1.0;
//
//    // === 狀態機定義 ===
//    // INTAKING: 正常吸球
//    // SETTLING: 球進入後等待，並決定下一步
//    // ALIGNING: 補救模式專用 (關閘 -> 轉動 -> 開閘)
//    // FULL:     全滿
//    private enum FillState { INTAKING, SETTLING, ALIGNING, FULL }
//    private enum FireState { IDLE, PREPARING, DECIDING, AIMING, KICKING, RETRACTING, RESETTING }
//
//    // === 運行時變量 ===
//    private FillState fillState = FillState.INTAKING;
//    private FireState fireState = FireState.IDLE;
//
//    private long fillTimer = 0;
//    private long fireTimer = 0;
//
//    // currentFillStep: 0=A, 1=B, 2=C
//    private int currentFillStep = 0;
//    private int targetRepairStep = 0; // 用於記錄補救的目標洞口
//
//    private boolean hasBallA = false, hasBallB = false, hasBallC = false;
//    private String colorHoleA = "EMPTY", colorHoleB = "EMPTY", colorHoleC = "EMPTY";
//
//    private double targetFirePos = 0;
//    private String currentTargetHole = "";
//
//    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }
//
//    @Override
//    public void runOpMode() {
//        initHardware();
//
//        DcMotor baseMotor = hardwareMap.get(DcMotor.class, "motor6");
//        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        Servo angleServo = hardwareMap.get(Servo.class, "servo3");
//        angleServo.setDirection(Servo.Direction.REVERSE);
//
//        telemetry.addData("Status", "Batch Fill V5 Ready");
//        telemetry.update();
//
//        double motorPowerVariable = 0;
//        long lastInputTime = 0;
//        long inputDelay = 200;
//
//        // 初始化狀態
//        fillState = FillState.INTAKING;
//        currentFillStep = 0; // 從 A 開始
//        diskServo.setPosition(FILL_POS_STEP_1);
//
//        waitForStart();
//
//        int b_upper_limit = 1895;
//        int b_lower_limit = 0;
//
//        while (opModeIsActive()) {
//            long currentTime = System.currentTimeMillis();
//
//            // 1. Shooter Control
//            if (gamepad1.dpad_left && (currentTime - lastInputTime > inputDelay)) {
//                motorPowerVariable += 0.1;
//                lastInputTime = currentTime;
//            } else if (gamepad1.dpad_right && (currentTime - lastInputTime > inputDelay)) {
//                motorPowerVariable -= 0.1;
//                lastInputTime = currentTime;
//            }
//            if (motorPowerVariable > 1.0) motorPowerVariable = 1.0;
//            if (motorPowerVariable < -1.0) motorPowerVariable = -1.0;
//            shooterMotor.setPower(motorPowerVariable);
//
//            // 2. Base Motor Control
//            double basePower = handlerange(gamepad1.left_trigger - gamepad1.right_trigger, 1, -1);
//            if (baseMotor.getCurrentPosition() >= b_upper_limit && basePower > 0) basePower = 0;
//            if (baseMotor.getCurrentPosition() <= b_lower_limit && basePower < 0) basePower = 0;
//            baseMotor.setPower(basePower);
//
//            // 3. Angle Servo
//            if (gamepad1.dpad_up) angleServo.setPosition(0.1314);
//            if (gamepad1.dpad_down) angleServo.setPosition(0);
//
//            // 4. Drive Control
//            double x = gamepad1.left_stick_x;
//            double y = -gamepad1.left_stick_y;
//            double rx = gamepad1.right_stick_x;
//            double theta = Math.atan2(y, x);
//            double power = Math.hypot(x, y);
//            double sin = Math.sin(theta - Math.PI / 4);
//            double cos = Math.cos(theta - Math.PI / 4);
//            double max = Math.max(Math.abs(sin), Math.abs(cos));
//
//            double fl = power * cos / max + rx;
//            double fr = power * sin / max - rx;
//            double bl = power * sin / max + rx;
//            double br = power * cos / max - rx;
//
//            if ((power + Math.abs(rx)) > 1) {
//                fl /= power + Math.abs(rx); fr /= power + Math.abs(rx);
//                bl /= power + Math.abs(rx); br /= power + Math.abs(rx);
//            }
//            frontLeftMotor.setPower(fl); backLeftMotor.setPower(bl);
//            frontRightMotor.setPower(fr); backRightMotor.setPower(br);
//
//            // 5. Fire Trigger
//            if (gamepad1.left_bumper && fireState == FireState.IDLE) {
//                if (hasBallA || hasBallB || hasBallC) {
//                    fireState = FireState.PREPARING;
//                    fireTimer = System.currentTimeMillis();
//                    controlGates(false);
//                }
//            }
//
//            // 6. Logic Execution
//            runFiringLogic();
//            runFillingLogic();
//            runIntakeLogic();
//
//            telemetry.addData("Fill State", fillState);
//            telemetry.addData("Current Hole", currentFillStep == 0 ? "A" : (currentFillStep == 1 ? "B" : "C"));
//            updateTelemetry();
//        }
//    }
//
//    // === 裝球邏輯 (A->B->C 快速流暢, 結束後檢查補洞) ===
//    private void runFillingLogic() {
//        if (fireState != FireState.IDLE) return;
//
//        switch (fillState) {
//            case INTAKING:
//                // 這是最常見的狀態：等待球進來
//                DetectedColor detectedColor = getDualSensorColor();
//                if (detectedColor != DetectedColor.UNKNOWN) {
//                    fillTimer = System.currentTimeMillis();
//                    fillState = FillState.SETTLING;
//                }
//                break;
//
//            case SETTLING:
//                // 等待球穩定
//                if (System.currentTimeMillis() - fillTimer > TIME_BALL_SETTLE) {
//                    // 確認球還在
//                    DetectedColor finalCheck = getDualSensorColor();
//                    if (finalCheck != DetectedColor.UNKNOWN) {
//                        recordBallColor(finalCheck);
//
//                        // === 關鍵邏輯修改處 ===
//                        // 1. 如果在 A (Step 0)，直接去 B (Step 1)
//                        if (currentFillStep == 0) {
//                            currentFillStep = 1;
//                            diskServo.setPosition(FILL_POS_STEP_2); // 快速移動，不關閘
//                            fillState = FillState.INTAKING;
//                        }
//                        // 2. 如果在 B (Step 1)，直接去 C (Step 2)
//                        else if (currentFillStep == 1) {
//                            currentFillStep = 2;
//                            diskServo.setPosition(FILL_POS_STEP_3); // 快速移動，不關閘
//                            fillState = FillState.INTAKING;
//                        }
//                        // 3. 如果在 C (Step 2)，檢查是否有漏掉的空位 (補救邏輯)
//                        else if (currentFillStep == 2) {
//                            checkAndRepairHoles();
//                        }
//                    } else {
//                        // 誤判或球彈出，繼續吸
//                        fillState = FillState.INTAKING;
//                    }
//                }
//                break;
//
//            case ALIGNING:
//                // 這是補救模式專用的狀態：關閘 -> 轉動 -> 開閘
//                double targetPos = FILL_POS_STEP_1;
//                if (targetRepairStep == 1) targetPos = FILL_POS_STEP_2;
//                if (targetRepairStep == 2) targetPos = FILL_POS_STEP_3;
//
//                diskServo.setPosition(targetPos);
//
//                // 等待轉動
//                if (System.currentTimeMillis() - fillTimer > TIME_DISK_MOVE) {
//                    currentFillStep = targetRepairStep; // 更新現在的位置
//                    controlGates(true);                 // 開閘
//                    fillState = FillState.INTAKING;     // 開始吸球
//                }
//                break;
//
//            case FULL:
//                if (!hasBallA || !hasBallB || !hasBallC) {
//                    // 如果滿球狀態下球掉了，重新檢查
//                    checkAndRepairHoles();
//                }
//                break;
//        }
//    }
//
//    // === 檢查是否有空洞並觸發補救 ===
//    private void checkAndRepairHoles() {
//        if (!hasBallA) {
//            // A 洞空了，需要補救 -> 觸發關閘轉動
//            targetRepairStep = 0;
//            triggerRepairMove();
//        } else if (!hasBallB) {
//            // B 洞空了，需要補救 -> 觸發關閘轉動
//            targetRepairStep = 1;
//            triggerRepairMove();
//        } else if (!hasBallC) {
//            // C 洞空了 (但在 Step 2 剛進球通常不會空，除非誤判)，留在此處繼續吸
//            targetRepairStep = 2;
//            currentFillStep = 2;
//            fillState = FillState.INTAKING;
//        } else {
//            // 全部都滿了
//            fillState = FillState.FULL;
//        }
//    }
//
//    private void triggerRepairMove() {
//        controlGates(false); // 1. 關閘 (保護)
//        fillTimer = System.currentTimeMillis();
//        fillState = FillState.ALIGNING; // 2. 進入對準狀態
//    }
//
//    // === 發射邏輯 (保持 C -> B -> A) ===
//    private void runFiringLogic() {
//        switch (fireState) {
//            case IDLE: break;
//            case PREPARING:
//                if (System.currentTimeMillis() - fireTimer > TIME_SHOOTER_SPIN) fireState = FireState.DECIDING;
//                break;
//            case DECIDING:
//                if (hasBallC) { targetFirePos = FIRE_POS_HOLE_C; currentTargetHole = "C"; switchToAiming(); }
//                else if (hasBallB) { targetFirePos = FIRE_POS_HOLE_B; currentTargetHole = "B"; switchToAiming(); }
//                else if (hasBallA) { targetFirePos = FIRE_POS_HOLE_A; currentTargetHole = "A"; switchToAiming(); }
//                else {
//                    fireTimer = System.currentTimeMillis();
//                    fireState = FireState.RESETTING;
//                    diskServo.setPosition(FILL_POS_STEP_1);
//                }
//                break;
//            case AIMING:
//                if (System.currentTimeMillis() - fireTimer > TIME_DISK_MOVE) {
//                    kickerServo.setPosition(KICKER_EXTEND);
//                    fireTimer = System.currentTimeMillis();
//                    fireState = FireState.KICKING;
//                }
//                break;
//            case KICKING:
//                if (System.currentTimeMillis() - fireTimer > TIME_KICK_OUT) {
//                    kickerServo.setPosition(KICKER_REST);
//                    clearBallStatus(currentTargetHole);
//                    fireTimer = System.currentTimeMillis();
//                    fireState = FireState.RETRACTING;
//                }
//                break;
//            case RETRACTING:
//                if (System.currentTimeMillis() - fireTimer > TIME_KICK_RETRACT) fireState = FireState.DECIDING;
//                break;
//            case RESETTING:
//                if (System.currentTimeMillis() - fireTimer > 600) {
//                    controlGates(true);
//                    currentFillStep = 0; // 發射完畢，重置回 A
//                    fillState = FillState.INTAKING;
//                    fireState = FireState.IDLE;
//                }
//                break;
//        }
//    }
//
//    private void switchToAiming() {
//        diskServo.setPosition(targetFirePos);
//        fireTimer = System.currentTimeMillis();
//        fireState = FireState.AIMING;
//    }
//
//    // === Intake 邏輯 ===
//    private void runIntakeLogic() {
//        if (fireState != FireState.IDLE) {
//            intakeMotor.setPower(0.0);
//            return;
//        }
//        // 只有在 "正在吸球" 或 "等待穩定" 時才開馬達
//        // ALIGNING (補救轉動中) 和 FULL 時關閉，避免卡球
//        if (fillState == FillState.INTAKING || fillState == FillState.SETTLING) {
//            intakeMotor.setPower(INTAKE_POWER);
//        } else {
//            intakeMotor.setPower(0.0);
//        }
//    }
//
//    private void recordBallColor(DetectedColor color) {
//        if (currentFillStep == 0) { colorHoleA = color.toString(); hasBallA = true; }
//        else if (currentFillStep == 1) { colorHoleB = color.toString(); hasBallB = true; }
//        else if (currentFillStep == 2) { colorHoleC = color.toString(); hasBallC = true; }
//    }
//
//    private void clearBallStatus(String hole) {
//        if (hole.equals("A")) { hasBallA = false; colorHoleA = "EMPTY"; }
//        if (hole.equals("B")) { hasBallB = false; colorHoleB = "EMPTY"; }
//        if (hole.equals("C")) { hasBallC = false; colorHoleC = "EMPTY"; }
//    }
//
//    private void controlGates(boolean isOpen) {
//        if (isOpen) { gateServoL.setPosition(GATE_L_OPEN); gateServoR.setPosition(GATE_R_OPEN); }
//        else { gateServoL.setPosition(GATE_CLOSED); gateServoR.setPosition(GATE_CLOSED); }
//    }
//
//    private DetectedColor getDualSensorColor() {
//        DetectedColor c1 = getDetectedColor(colorSensor1);
//        DetectedColor c2 = getDetectedColor(colorSensor2);
//        if (c1 != DetectedColor.UNKNOWN) return c1;
//        if (c2 != DetectedColor.UNKNOWN) return c2;
//        return DetectedColor.UNKNOWN;
//    }
//
//    public DetectedColor getDetectedColor(NormalizedColorSensor sensor) {
//        NormalizedRGBA color = sensor.getNormalizedColors();
//        if (color.alpha < MIN_DETECT_BRIGHTNESS) return DetectedColor.UNKNOWN;
//        if (color.blue > color.green && color.blue > color.red) {
//            if (color.blue > (color.green * PURPLE_RATIO_LIMIT)) return DetectedColor.PURPLE;
//        }
//        if (color.green > color.red) {
//            if (color.green >= color.blue || (color.green > color.blue * 0.85f)) return DetectedColor.GREEN;
//        }
//        return DetectedColor.UNKNOWN;
//    }
//
//    private void initHardware() {
//        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
//        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
//        if (colorSensor1 instanceof com.qualcomm.robotcore.hardware.SwitchableLight) ((com.qualcomm.robotcore.hardware.SwitchableLight) colorSensor1).enableLight(true);
//        if (colorSensor2 instanceof com.qualcomm.robotcore.hardware.SwitchableLight) ((com.qualcomm.robotcore.hardware.SwitchableLight) colorSensor2).enableLight(true);
//        colorSensor1.setGain(SENSOR_GAIN); colorSensor2.setGain(SENSOR_GAIN);
//
//        kickerServo = hardwareMap.get(Servo.class, "servo1");
//        diskServo = hardwareMap.get(Servo.class, "servo2");
//        gateServoL = hardwareMap.get(Servo.class, "servo4");
//        gateServoR = hardwareMap.get(Servo.class, "servo5");
//
//        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
//        shooterMotor = hardwareMap.get(DcMotor.class, "motor5");
//        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
//        backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
//        frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
//        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
//
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        kickerServo.scaleRange(0.0, 0.5);
//        gateServoL.setDirection(Servo.Direction.REVERSE);
//        gateServoR.setDirection(Servo.Direction.FORWARD);
//        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        kickerServo.setPosition(KICKER_REST);
//        diskServo.setPosition(FILL_POS_STEP_1);
//        controlGates(true);
//        intakeMotor.setPower(0);
//        shooterMotor.setPower(0);
//
//        fillState = FillState.INTAKING;
//        currentFillStep = 0;
//    }
//
//    private void updateTelemetry() {
//        telemetry.addLine("=== SYSTEM STATUS ===");
//        telemetry.addData("Fill State", fillState);
//        telemetry.addData("Current Hole", currentFillStep == 0 ? "A" : (currentFillStep == 1 ? "B" : "C"));
//
//        if (fillState == FillState.ALIGNING) telemetry.addData("Action", "REPAIRING (Gate Closed)");
//        else if (fireState != FireState.IDLE) telemetry.addData("Action", "FIRING");
//        else telemetry.addData("Action", "Normal Operation");
//
//        telemetry.addLine("\n=== BALLS ===");
//        telemetry.addData("A", "[%s] %s", colorHoleA, hasBallA ? "●" : "○");
//        telemetry.addData("B", "[%s] %s", colorHoleB, hasBallB ? "●" : "○");
//        telemetry.addData("C", "[%s] %s", colorHoleC, hasBallC ? "●" : "○");
//        telemetry.update();
//    }
//}