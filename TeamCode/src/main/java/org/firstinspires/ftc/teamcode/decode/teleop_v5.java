package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop_v5")
public class teleop_v5 extends LinearOpMode {

    // === 硬件變量 ===
    NormalizedColorSensor colorSensor;

    // Servos
    Servo kickerServo;  // servo1: 長棍 (踢球用)
    Servo diskServo;    // servo2: 圓盤 (轉動用)
    Servo gateServoL;   // servo4: 左閘門
    Servo gateServoR;   // servo5: 右閘門

    // Motors
    DcMotor intakeMotor;  // motor4: 進球馬達 (常開)
    DcMotor shooterMotor; // motor5: 發射馬達 (發射時開)
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;


    // === 參數設定 ===
    // 圓盤位置 (Filling)
    private static final double FILL_POS_STEP_1 = 0.0;  // Hole A @ Sensor
    private static final double FILL_POS_STEP_2 = 0.3529; // Hole B @ Sensor
    private static final double FILL_POS_STEP_3 = 0.7137; // Hole C @ Sensor

    // 圓盤位置 (Firing)
    private static final double FIRE_POS_HOLE_B = 0.0471;
    private static final double FIRE_POS_HOLE_C = 0.4314;
    private static final double FIRE_POS_HOLE_A = 0.8196;

    // 長棍 (Kicker) 邏輯位置 (物理範圍已被限制在 0.0-0.5)
    private static final double KICKER_REST = 0.0;
    private static final double KICKER_EXTEND = 0.8;
    private static final int KICKER_WAIT_MS = 300;

    // 閘門 (Gate) 位置
    // 0.0 = 開閘 (球可以進入)
    // 1.0 = 關閘 (球不會飛出)
    private static final double GATE_OPEN = 0.0;
    private static final double GATE_CLOSED = 1.0;

    // 傳感器
    private static final float SENSOR_GAIN = 25.0f;
    private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
    private static final float PURPLE_RATIO_LIMIT = 1.2f;

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

        telemetry.addData("Status", "System Initialized");
        telemetry.addData("Gate Status", "OPEN (Pos: 0.0)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
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


            // === 1. 發射模式控制 (Left Bumper) ===
            if (gamepad1.left_bumper && !isFiringMode) {
                // 如果還有球，才允許發射
                if (hasBallA || hasBallB || hasBallC) {
                    performFiringSequence();
                }
            }

            // === 2. 進球馬達控制 (Motor 4) ===
            // 邏輯：如果沒滿 且 不在發射模式 -> 開啟馬達吸球
            // 否則 (滿了 或 正在射) -> 關閉馬達
            if (currentFillStep < 3 && !isFiringMode) {
                intakeMotor.setPower(-1); // 吸球速度
            } else {
                intakeMotor.setPower(0.0);
            }

            // === 3. 自動裝球邏輯 (Filling) ===
            // 只有在非發射模式 且 沒滿的時候執行
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
        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // --- Servo 設定 ---
        // 1. Kicker (長棍) 範圍限制
        kickerServo.scaleRange(0.0, 0.5);

        // 2. Gate (閘門) 方向修正 [更新部分]
        // 根據您的反饋，將兩個方向反轉
        gateServoL.setDirection(Servo.Direction.FORWARD); // 修正：左邊改為 Forward
        gateServoR.setDirection(Servo.Direction.REVERSE); // 修正：右邊改為 Reverse

        // 初始狀態
        kickerServo.setPosition(KICKER_REST);
        diskServo.setPosition(FILL_POS_STEP_1);

        // 閘門初始開啟 (為了進球)
        controlGates(true); // Open

        // 馬達初始關閉
        intakeMotor.setPower(0);
        shooterMotor.setPower(0);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // === 核心邏輯：裝球 ===
    private void handleFillingLogic() {
        DetectedColor detectedColor = getDetectedColor(colorSensor);

        if (detectedColor != DetectedColor.UNKNOWN) {
            // 檢測到球

            // 1. 先記錄
            switch (currentFillStep) {
                case 0:
                    colorHoleA = detectedColor.toString();
                    hasBallA = true;
                    telemetry.addData("Event", "Got Ball A -> Moving");
                    telemetry.update();
                    // 轉動到 Step 2
                    safeRotateDisk(FILL_POS_STEP_2);
                    currentFillStep = 1;
                    break;

                case 1:
                    colorHoleB = detectedColor.toString();
                    hasBallB = true;
                    telemetry.addData("Event", "Got Ball B -> Moving");
                    telemetry.update();
                    // 轉動到 Step 3
                    safeRotateDisk(FILL_POS_STEP_3);
                    currentFillStep = 2;
                    break;

                case 2:
                    colorHoleC = detectedColor.toString();
                    hasBallC = true;
                    telemetry.addData("Event", "Got Ball C -> Full");
                    telemetry.update();
                    // 滿了，不轉動
                    currentFillStep = 3;
                    break;
            }
            // 等待球穩定，避免重複讀取
            sleep(800);
        }
    }

    // === 核心邏輯：發射序列 ===
    private void performFiringSequence() {
        isFiringMode = true; // 標記為發射模式 (自動關閉 Intake Motor)

        // 1. 啟動發射馬達 (Motor 5)
        shooterMotor.setPower(0.8);
        telemetry.addData("Mode", "SHOOTER SPINNING UP...");
        telemetry.update();

        // 關閉閘門 (防止球在發射移動過程中飛出)
        controlGates(false); // Close

        sleep(1000); // 等待發射輪達到速度

        // 2. 循環發射直到沒球
        while (opModeIsActive() && (hasBallA || hasBallB || hasBallC)) {

            double currentPos = diskServo.getPosition();

            // 計算距離
            double distA = hasBallA ? Math.abs(currentPos - FIRE_POS_HOLE_A) : 999.0;
            double distB = hasBallB ? Math.abs(currentPos - FIRE_POS_HOLE_B) : 999.0;
            double distC = hasBallC ? Math.abs(currentPos - FIRE_POS_HOLE_C) : 999.0;

            // 選擇最短路徑發射
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

        // 3. 發射結束清理
        shooterMotor.setPower(0.0); // 關閉發射馬達

        telemetry.addData("Mode", "FIRING DONE - RESETTING");
        telemetry.update();

        // 回到初始位置
        diskServo.setPosition(FILL_POS_STEP_1);
        sleep(600);

        // 打開閘門 (準備重新進球)
        controlGates(true); // Open

        currentFillStep = 0;
        isFiringMode = false; // 解除發射模式 (Intake Motor 會自動重啟)
    }

    // === 動作輔助方法 ===

    // 單次射擊動作
    private void shootOneBall(double targetPos, String holeName) {
        telemetry.addData("Action", "Shooting Hole " + holeName);
        telemetry.update();

        // 轉動圓盤 (閘門保持關閉)
        diskServo.setPosition(targetPos);
        sleep(500); // 等待到位

        // Kicker 動作 (長棍)
        kickerServo.setPosition(KICKER_EXTEND);
        sleep(KICKER_WAIT_MS);
        kickerServo.setPosition(KICKER_REST);
        sleep(250);
    }

    // 安全轉動圓盤 (用於裝球階段)
    private void safeRotateDisk(double targetPos) {
        // 1. 關閘門 (安全)
        controlGates(false); // Close
        sleep(200);

        // 2. 轉動
        diskServo.setPosition(targetPos);
        sleep(500);

        // 3. 開閘門 (進球)
        controlGates(true); // Open
        sleep(200);
    }

    // 控制閘門開關 (isOpen: true=開/0, false=關/1)
    private void controlGates(boolean isOpen) {
        double pos = isOpen ? GATE_OPEN : GATE_CLOSED;
        gateServoL.setPosition(pos);
        gateServoR.setPosition(pos);
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

    // 用於顯示的 Telemetry
    private void updateTelemetry() {
        telemetry.addLine("=== STATUS ===");
        if (isFiringMode) {
            telemetry.addData("State", "FIRING MODE (Motor5 ON)");
        } else if (currentFillStep >= 3) {
            telemetry.addData("State", "FULL (Intake OFF)");
        } else {
            telemetry.addData("State", "FILLING (Intake ON)");
        }

        telemetry.addLine("\n=== BALLS ===");
        telemetry.addData("A", "[%s] %s", colorHoleA, hasBallA?"●":"○");
        telemetry.addData("B", "[%s] %s", colorHoleB, hasBallB?"●":"○");
        telemetry.addData("C", "[%s] %s", colorHoleC, hasBallC?"●":"○");

        telemetry.update();
    }
}