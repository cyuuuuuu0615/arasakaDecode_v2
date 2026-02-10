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

@TeleOp(name = "A BULE Teleop")
public class teleop_v3 extends LinearOpMode {

     private Limelight3A limelight;
    private final double TARGET_TX = 6.0;
    private final double MIN_POWER = 0.06;
    private double lastError = 0;
    private double KP = 0.011;
    private double KD = 0.095;
    private final double MAX_POWER = 0.40;
    private final double DEADBAND = 1.5;


    private static final double CAMERA_HEIGHT = 14.5;
    private static final double TARGET_HEIGHT = 39.0;
    private static final double MOUNT_ANGLE = 17.8;
    private static final double DISTANCE_THRESHOLD = 35.0;
    private static final double ANGLE_CLOSE = 0.0;
    private static final double ANGLE_FAR = 0.12;

    private static final double RPM_SLOPE_CLOSE = 11.0;
    private static final double RPM_BASE_CLOSE = 610.0;
    private static final double RPM_SLOPE_FAR = 11.0;  // 原 10.0 -> 11.0 (距離加成更強)
    private static final double RPM_BASE_FAR = 680.0;  // 原 620.0 -> 680.0 (基礎轉速+60)
    private static final double RPM_IDLE = 300.0;

    private double currentCommandedRpm = RPM_IDLE;
    private static final double RPM_RAMP_DOWN_STEP = 5.0;

    NormalizedColorSensor colorSensor1, colorSensor2;
    Servo kickerServo, diskServo, gateServoL, gateServoR, angleServo;
    DcMotor intakeMotor, baseMotor;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    DcMotorEx shooterMotorLeft, shooterMotorRight;
    private LED LED0, LED1, LED2;

    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(92, 0, 0, 15);

    private static final double FILL_POS_STEP_1 = 0.0;    // Hole A
    private static final double FILL_POS_STEP_2 = 0.3529; // Hole B
    private static final double FILL_POS_STEP_3 = 0.7137; // Hole C

    private static final double FIRE_POS_HOLE_B = 0.0471;
    private static final double FIRE_POS_HOLE_C = 0.4314;
    private static final double FIRE_POS_HOLE_A = 0.8196;

    private static final double KICKER_REST = 0.0;
    private static final double KICKER_EXTEND = 0.8;

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
    private boolean lastButtonA = false;

    @Override
    public void runOpMode() {
        initHardware();

        try {
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            telemetry.addData("Limelight Error", "Init Failed: " + e.getMessage());
        }

        angleServo.setDirection(Servo.Direction.REVERSE);
        angleServo.setPosition(ANGLE_CLOSE);

        telemetry.addData("Status", "Ready (Smart Refill Logic Active)");
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
                if (result != null && result.isValid()) targetValid = true;
            } catch (Exception e) { targetValid = false; }


            if (targetValid) {
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


            boolean currentButtonA = gamepad1.a;
            if (currentButtonA && !lastButtonA) {
                if (currentFillStep > 0 && fireState == FireState.IDLE) {
                    currentFillStep--;
                    if (currentFillStep == 0) {
                        diskServo.setPosition(FILL_POS_STEP_1);
                        clearBallStatus("A");
                        LED0.off();
                    } else if (currentFillStep == 1) {
                        diskServo.setPosition(FILL_POS_STEP_2);
                        clearBallStatus("B");
                        LED1.off();
                    } else if (currentFillStep == 2) {
                        diskServo.setPosition(FILL_POS_STEP_3);
                        clearBallStatus("C");
                        LED2.off();
                    }
                    fillState = FillState.IDLE;
                    controlGates(true); // 手動回退時也要開閘
                }
            }
            lastButtonA = currentButtonA;


            if (gamepad1.left_bumper && fireState == FireState.IDLE) {
                if (hasBallA || hasBallB || hasBallC) {
                    fireState = FireState.PREPARING;
                    fireTimer = System.currentTimeMillis();
                    controlGates(false);
                }
            }
            runFiringLogic();
            runFillingLogic();

            if(gamepad1.y) {
                intakeMotor.setPower(-1);
            } else {
                runIntakeLogic();
            }

            if (targetValid) {
                telemetry.addData("Limelight", "Tx: %.2f | Dist: %.1f", result.getTx(), currentDistance);
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

    private void runFillingLogic() {
        if (fireState != FireState.IDLE) return;

        if (currentFillStep >= 3) {
            if (!hasBallA) {
                currentFillStep = 0;
                diskServo.setPosition(FILL_POS_STEP_1);
                controlGates(true);
                fillState = FillState.IDLE;
                return;
            }
            else if (!hasBallB) {
                currentFillStep = 1;
                diskServo.setPosition(FILL_POS_STEP_2);
                controlGates(true);
                fillState = FillState.IDLE;
                return;
            }
            else if (!hasBallC) {
                currentFillStep = 2;
                diskServo.setPosition(FILL_POS_STEP_3);
                controlGates(true);
                fillState = FillState.IDLE;
                return;
            }

            fillState = FillState.FULL;
            controlGates(false);
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
            case FULL:
                break;
        }
    }

    private void runFiringLogic() { switch (fireState) { case IDLE: break; case PREPARING: if (System.currentTimeMillis() - fireTimer > TIME_SHOOTER_SPIN) fireState = FireState.DECIDING; break; case DECIDING: if (hasBallC) { targetFirePos = FIRE_POS_HOLE_C; currentTargetHole = "C"; switchToAiming(); LED2.off(); } else if (hasBallB) { targetFirePos = FIRE_POS_HOLE_B; currentTargetHole = "B"; switchToAiming(); LED1.off(); } else if (hasBallA) { targetFirePos = FIRE_POS_HOLE_A; currentTargetHole = "A"; switchToAiming(); LED0.off(); } else { fireTimer = System.currentTimeMillis(); fireState = FireState.RESETTING; diskServo.setPosition(FILL_POS_STEP_1); } break; case AIMING: if (System.currentTimeMillis() - fireTimer > TIME_DISK_MOVE_SHOOTING) { kickerServo.setPosition(KICKER_EXTEND); fireTimer = System.currentTimeMillis(); fireState = FireState.KICKING; } break; case KICKING: if (System.currentTimeMillis() - fireTimer > TIME_KICK_OUT) { kickerServo.setPosition(KICKER_REST); clearBallStatus(currentTargetHole); fireTimer = System.currentTimeMillis(); fireState = FireState.RETRACTING; } break; case RETRACTING: if (System.currentTimeMillis() - fireTimer > TIME_KICK_RETRACT) fireState = FireState.DECIDING; break; case RESETTING: if (System.currentTimeMillis() - fireTimer > 600) { controlGates(true); currentFillStep = 0; fireState = FireState.IDLE; isHighSpeedMode = false; } break; } }
    private void switchToAiming() { diskServo.setPosition(targetFirePos); fireTimer = System.currentTimeMillis(); fireState = FireState.AIMING; }
    private void runIntakeLogic() { if (currentFillStep < 3 && fireState == FireState.IDLE) { intakeMotor.setPower(INTAKE_POWER); } else { intakeMotor.setPower(0.0); } }
    private void recordBallColor(DetectedColor color) { switch (currentFillStep) { case 0: colorHoleA = color.toString(); hasBallA = true; LED0.on(); break; case 1: colorHoleB = color.toString(); hasBallB = true; LED1.on(); break; case 2: colorHoleC = color.toString(); hasBallC = true; LED2.on(); break; } }

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
        telemetry.addLine("=== SYSTEM STATUS ===");
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