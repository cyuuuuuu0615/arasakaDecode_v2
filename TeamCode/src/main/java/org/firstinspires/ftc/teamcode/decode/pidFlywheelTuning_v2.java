package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "pidFlywheelTuning_v2")
public class pidFlywheelTuning_v2 extends OpMode {

    // --- Hardware ---
    public DcMotorEx shooterMotorRight;
    public DcMotor shooterMotorLeft, intakeMotor, baseMotor;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public Servo kickerServo, diskServo, gateServoL, gateServoR, angleServo;
    public NormalizedColorSensor colorSensor1, colorSensor2;

    // --- PID Tuning Variables ---
    public double highVelocity = 1500;
    public double lowVelocity = 900;
    double curTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    // --- State Logic Flags ---
    private boolean lastY = false, lastB = false, lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

    // --- Limits and Constants ---
    private int b_upper_limit = 1000, b_lower_limit = 0;
    private static final double FILL_POS_STEP_1 = 0.0, FILL_POS_STEP_2 = 0.3529, FILL_POS_STEP_3 = 0.7137;
    private static final double FIRE_POS_HOLE_A = 0.8196, FIRE_POS_HOLE_B = 0.0471, FIRE_POS_HOLE_C = 0.4314;
    private static final double KICKER_REST = 0.0, KICKER_EXTEND = 0.8;
    private static final double GATE_CLOSED = 0.0, GATE_L_OPEN = 0.6667, GATE_R_OPEN = 0.6902;
    private static final int TIME_BALL_SETTLE = 50, TIME_DISK_MOVE_INTAKE = 250, TIME_DISK_MOVE_SHOOTING = 500;
    private static final int TIME_SHOOTER_SPIN = 1000, TIME_KICK_OUT = 300, TIME_KICK_RETRACT = 250;
    private static final float SENSOR_GAIN = 25.0f, MIN_DETECT_BRIGHTNESS = 0.7f, PURPLE_RATIO_LIMIT = 1.2f;
    private static final double INTAKE_POWER = 0.6;

    // --- Enums and State ---
    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }
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

    @Override
    public void init() {
        // Motors
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "motor7");
        shooterMotorLeft = hardwareMap.get(DcMotor.class, "motor5");
        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
        baseMotor = hardwareMap.get(DcMotor.class, "motor6");


        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        kickerServo = hardwareMap.get(Servo.class, "servo1");
        diskServo = hardwareMap.get(Servo.class, "servo2");
        angleServo = hardwareMap.get(Servo.class, "servo3");
        gateServoL = hardwareMap.get(Servo.class, "servo4");
        gateServoR = hardwareMap.get(Servo.class, "servo5");

        kickerServo.scaleRange(0.0, 0.5);
        gateServoL.setDirection(Servo.Direction.REVERSE);

        // Sensors
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        colorSensor1.setGain(SENSOR_GAIN);
        colorSensor2.setGain(SENSOR_GAIN);

        kickerServo.setPosition(KICKER_REST);
        diskServo.setPosition(FILL_POS_STEP_1);
        controlGates(true);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        // --- 1. Base Motor Control ---
        double basePower = (-gamepad1.left_trigger + gamepad1.right_trigger) * 0.5;
        int pos = baseMotor.getCurrentPosition();
        if ((pos >= b_upper_limit && basePower > 0) || (pos <= b_lower_limit && basePower < 0)) {
            baseMotor.setPower(0);
        } else {
            baseMotor.setPower(basePower);
        }

        // --- 2. Servo Angle Control ---
        if (gamepad1.dpad_up) angleServo.setPosition(0.2);
        if (gamepad1.dpad_down) angleServo.setPosition(0);
        if (gamepad1.a) angleServo.setPosition(0.1);

        // --- 3. PID Tuning Input Handling (Rising Edge Detection) ---
        if (gamepad1.y && !lastY) curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
        if (gamepad1.b && !lastB) stepIndex = (stepIndex + 1) % stepSizes.length;
        if (gamepad1.dpad_left && !lastLeft) F -= stepSizes[stepIndex];
        if (gamepad1.dpad_right && !lastRight) F += stepSizes[stepIndex];
        if (gamepad1.dpad_up && !lastUp) P += stepSizes[stepIndex];
        if (gamepad1.dpad_down && !lastDown) P -= stepSizes[stepIndex];

        lastY = gamepad1.y; lastB = gamepad1.b; lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down; lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;

        // --- 4. Apply PIDF & Velocity ---
        shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));
        shooterMotorRight.setVelocity(curTargetVelocity);


        // --- 5. Automation Logic ---
        if (gamepad1.left_bumper && fireState == FireState.IDLE) {
            if (hasBallA || hasBallB || hasBallC) {
                fireState = FireState.PREPARING;
                fireTimer = System.currentTimeMillis();
                controlGates(false);
            }
        }

        runFiringLogic();
        runFillingLogic();

        if (gamepad1.y) intakeMotor.setPower(-1); // Manual outtake
        else runIntakeLogic();

        // --- 6. Telemetry ---
        telemetry.addData("Target Vel", curTargetVelocity);
        telemetry.addData("Current Vel", "%.2f", shooterMotorRight.getVelocity());
        telemetry.addData("P", "%.4f", P);
        telemetry.addData("F", "%.4f", F);
        updateTelemetry();
    }

    private void runFillingLogic() {
        if (fireState != FireState.IDLE) return;
        if (currentFillStep >= 3) { fillState = FillState.FULL; return; }

        switch (fillState) {
            case IDLE:
                DetectedColor detected = getDualSensorColor();
                if (detected != DetectedColor.UNKNOWN) {
                    recordBallColor(detected);
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
            case FULL:
                if (currentFillStep < 3) fillState = FillState.IDLE;
                break;
        }
    }

    private void runFiringLogic() {
        switch (fireState) {
            case IDLE: break;
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
        else { currentFillStep = 3; }
    }

    private void clearBallStatus(String hole) {
        if (hole.equals("A")) { hasBallA = false; colorHoleA = "EMPTY"; }
        else if (hole.equals("B")) { hasBallB = false; colorHoleB = "EMPTY"; }
        else if (hole.equals("C")) { hasBallC = false; colorHoleC = "EMPTY"; }
    }

    private void controlGates(boolean isOpen) {
        double posL = isOpen ? GATE_L_OPEN : GATE_CLOSED;
        double posR = isOpen ? GATE_R_OPEN : GATE_CLOSED;
        gateServoL.setPosition(posL);
        gateServoR.setPosition(posR);
    }

    private DetectedColor getDualSensorColor() {
        DetectedColor c1 = getDetectedColor(colorSensor1);
        if (c1 != DetectedColor.UNKNOWN) return c1;
        return getDetectedColor(colorSensor2);
    }

    public DetectedColor getDetectedColor(NormalizedColorSensor sensor) {
        NormalizedRGBA color = sensor.getNormalizedColors();
        if (color.alpha < MIN_DETECT_BRIGHTNESS) return DetectedColor.UNKNOWN;
        if (color.blue > color.green && color.blue > (color.green * PURPLE_RATIO_LIMIT)) return DetectedColor.PURPLE;
        if (color.green > color.red && (color.green >= color.blue || color.green > color.blue * 0.85f)) return DetectedColor.GREEN;
        return DetectedColor.UNKNOWN;
    }

    private void updateTelemetry() {
        telemetry.addData("Fill/Fire", "%s | %s", fillState, fireState);
        telemetry.addData("Balls", "A:%b B:%b C:%b", hasBallA, hasBallB, hasBallC);
        telemetry.update();
    }
}