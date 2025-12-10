package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple; // 引入此項以使用 Direction

@TeleOp(name = "teleop_v4")
public class teleop_v4 extends LinearOpMode {

    // === 硬件變量 ===
    NormalizedColorSensor colorSensor;

    // Servos
    Servo kickerServo;  // servo1: 長棍 (踢球用)
    Servo diskServo;    // servo2: 圓盤 (轉動用)
    Servo gateServoL;   // servo4: 左閘門
    Servo gateServoR;   // servo5: 右閘門

    // Motors
    DcMotor intakeMotor;  // motor4: 進球馬達
    DcMotor shooterMotor; // motor5: 發射馬達

    // === 參數設定 ===

    // 圓盤位置 (Filling - 裝球)
    private static final double FILL_POS_STEP_1 = 0.0;     // Hole A @ Sensor
    private static final double FILL_POS_STEP_2 = 0.3529;  // Hole B @ Sensor
    private static final double FILL_POS_STEP_3 = 0.7137;  // Hole C @ Sensor

    // 圓盤位置 (Firing - 發射)
    private static final double FIRE_POS_HOLE_B = 0.0471;
    private static final double FIRE_POS_HOLE_C = 0.4314;
    private static final double FIRE_POS_HOLE_A = 0.8196;

    // 長棍 (Kicker)
    private static final double KICKER_REST = 0.0;
    private static final double KICKER_EXTEND = 0.8;
    private static final int KICKER_WAIT_MS = 300;

    // 閘門 (Gate) 位置參數
    private static final double GATE_CLOSED = 0.0;
    private static final double GATE_L_OPEN = 0.6667; // Servo 4 Open
    private static final double GATE_R_OPEN = 0.6902; // Servo 5 Open

    // 傳感器
    private static final float SENSOR_GAIN = 25.0f;
    private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
    private static final float PURPLE_RATIO_LIMIT = 1.2f;

    // Intake Power
    private static final double INTAKE_POWER = -1.0;

    // === 狀態變量 ===
    private int currentFillStep = 0; // 0, 1, 2, 3(Full)
    private boolean isFiringMode = false; // 是否正在發射模式中

    // 球體狀態
    private boolean hasBallA = false;
    private boolean hasBallB = false;
    private boolean hasBallC = false;

    // 顏色紀錄
    private String colorHoleA = "EMPTY";
    private String colorHoleB = "EMPTY";
    private String colorHoleC = "EMPTY";

    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }

    @Override
    public void runOpMode() {
        // 1. 初始化硬件
        initHardware();

        telemetry.addData("Status", "teleop_v5 Initialized");
        telemetry.addData("Shooter Dir", "REVERSE");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // === 1. 發射模式控制 (Left Bumper) ===
            if (gamepad1.left_bumper && !isFiringMode) {
                if (hasBallA || hasBallB || hasBallC) {
                    performFiringSequence();
                }
            }

            // === 2. 進球馬達控制 ===
            if (currentFillStep < 3 && !isFiringMode) {
                intakeMotor.setPower(INTAKE_POWER);
            } else {
                intakeMotor.setPower(0.0);
            }

            // === 3. 自動裝球邏輯 ===
            if (!isFiringMode && currentFillStep < 3) {
                handleFillingLogic();
            }

            // === 4. 更新顯示 ===
            updateTelemetry();
        }
    }

    // === 初始化方法 ===
    private void initHardware() {
        // Sensors
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        if (colorSensor instanceof com.qualcomm.robotcore.hardware.SwitchableLight) {
            ((com.qualcomm.robotcore.hardware.SwitchableLight)colorSensor).enableLight(true);
        }
        colorSensor.setGain(SENSOR_GAIN);

        // Servos
        kickerServo = hardwareMap.get(Servo.class, "servo1");
        diskServo = hardwareMap.get(Servo.class, "servo2");
        gateServoL = hardwareMap.get(Servo.class, "servo4");
        gateServoR = hardwareMap.get(Servo.class, "servo5");

        // Motors
        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
        shooterMotor = hardwareMap.get(DcMotor.class, "motor5");

        // --- Servo 設定 ---
        // 1. Kicker 範圍限制
        kickerServo.scaleRange(0.0, 0.5);

        // 2. Gate 方向設定
        gateServoL.setDirection(Servo.Direction.REVERSE); // Servo 4: Reverse
        gateServoR.setDirection(Servo.Direction.FORWARD); // Servo 5: Forward (Normal)

        // --- Motor 設定 ---
        // [更新] Shooter 方向反轉
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 初始狀態
        kickerServo.setPosition(KICKER_REST);
        diskServo.setPosition(FILL_POS_STEP_1);

        // 閘門初始開啟
        controlGates(true); // Open

        // 馬達初始關閉
        intakeMotor.setPower(0);
        shooterMotor.setPower(0);
    }

    // === 核心邏輯：裝球 ===
    private void handleFillingLogic() {
        DetectedColor detectedColor = getDetectedColor(colorSensor);

        if (detectedColor != DetectedColor.UNKNOWN) {
            switch (currentFillStep) {
                case 0:
                    colorHoleA = detectedColor.toString();
                    hasBallA = true;
                    telemetry.addData("Event", "Got Ball A -> Moving");
                    telemetry.update();
                    rotateDiskSimple(FILL_POS_STEP_2);
                    currentFillStep = 1;
                    break;

                case 1:
                    colorHoleB = detectedColor.toString();
                    hasBallB = true;
                    telemetry.addData("Event", "Got Ball B -> Moving");
                    telemetry.update();
                    rotateDiskSimple(FILL_POS_STEP_3);
                    currentFillStep = 2;
                    break;

                case 2:
                    colorHoleC = detectedColor.toString();
                    hasBallC = true;
                    telemetry.addData("Event", "Got Ball C -> Full");
                    telemetry.update();
                    currentFillStep = 3;
                    break;
            }
            sleep(800);
        }
    }

    // === 核心邏輯：發射序列 ===
    private void performFiringSequence() {
        isFiringMode = true;

        // 1. 關閉閘門
        controlGates(false);

        // 2. 啟動發射馬達 (注意: 方向已在 init 中反轉，這裡設正數即可)
        shooterMotor.setPower(0.8);
        telemetry.addData("Mode", "FIRING: Gates Closed");
        telemetry.update();

        sleep(1000);

        // 3. 循環發射
        while (opModeIsActive() && (hasBallA || hasBallB || hasBallC)) {
            double currentPos = diskServo.getPosition();

            double distA = hasBallA ? Math.abs(currentPos - FIRE_POS_HOLE_A) : 999.0;
            double distB = hasBallB ? Math.abs(currentPos - FIRE_POS_HOLE_B) : 999.0;
            double distC = hasBallC ? Math.abs(currentPos - FIRE_POS_HOLE_C) : 999.0;

            if (distA <= distB && distA <= distC) {
                shootOneBall(FIRE_POS_HOLE_A, "A");
                hasBallA = false;
                colorHoleA = "EMPTY";
            } else if (distB <= distA && distB <= distC) {
                shootOneBall(FIRE_POS_HOLE_B, "B");
                hasBallB = false;
                colorHoleB = "EMPTY";
            } else {
                shootOneBall(FIRE_POS_HOLE_C, "C");
                hasBallC = false;
                colorHoleC = "EMPTY";
            }
        }

        // 4. 發射結束清理
        shooterMotor.setPower(0.0);

        telemetry.addData("Mode", "FIRING DONE - RESETTING");
        telemetry.update();

        diskServo.setPosition(FILL_POS_STEP_1);
        sleep(600);

        // 重新打開閘門
        controlGates(true); // Open

        currentFillStep = 0;
        isFiringMode = false;
    }

    // === 動作輔助方法 ===
    private void shootOneBall(double targetPos, String holeName) {
        telemetry.addData("Action", "Shooting Hole " + holeName);
        telemetry.update();

        diskServo.setPosition(targetPos);
        sleep(500);

        kickerServo.setPosition(KICKER_EXTEND);
        sleep(KICKER_WAIT_MS);
        kickerServo.setPosition(KICKER_REST);
        sleep(250);
    }

    private void rotateDiskSimple(double targetPos) {
        diskServo.setPosition(targetPos);
        sleep(500);
    }

    // 控制閘門開關
    private void controlGates(boolean isOpen) {
        if (isOpen) {
            gateServoL.setPosition(GATE_L_OPEN); // 0.6667
            gateServoR.setPosition(GATE_R_OPEN); // 0.6902
        } else {
            gateServoL.setPosition(GATE_CLOSED); // 0.0
            gateServoR.setPosition(GATE_CLOSED); // 0.0
        }
    }

    // 顏色檢測
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

    private void updateTelemetry() {
        telemetry.addLine("=== STATUS ===");
        if (isFiringMode) {
            telemetry.addData("State", "FIRING MODE (Gates Closed)");
        } else if (currentFillStep >= 3) {
            telemetry.addData("State", "FULL (Intake OFF)");
        } else {
            telemetry.addData("State", "FILLING (Intake ON: -1.0)");
        }
        telemetry.addData("Balls", "A:%s B:%s C:%s", hasBallA?"●":"○", hasBallB?"●":"○", hasBallC?"●":"○");
        telemetry.update();
    }
}