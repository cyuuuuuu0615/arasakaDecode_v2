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
    Servo kickerServo, diskServo, gateServoL, gateServoR;
    DcMotor intakeMotor, shooterMotor;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;


    // === 參數設定 (維持不變) ===
    private static final double FILL_POS_STEP_1 = 0.0;
    private static final double FILL_POS_STEP_2 = 0.3529;
    private static final double FILL_POS_STEP_3 = 0.7137;

    private static final double FIRE_POS_HOLE_B = 0.0471;
    private static final double FIRE_POS_HOLE_C = 0.4314;
    private static final double FIRE_POS_HOLE_A = 0.8196;

    private static final double KICKER_REST = 0.0;
    private static final double KICKER_EXTEND = 0.8;

    // 時間參數 (ms)
    private static final int TIME_BALL_SETTLE = 800;  // 球穩定的時間
    private static final int TIME_DISK_MOVE = 500;    // 圓盤轉動時間
    private static final int TIME_SHOOTER_SPIN = 1000;// 發射輪加速時間
    private static final int TIME_KICK_OUT = 300;     // 踢出時間
    private static final int TIME_KICK_RETRACT = 250; // 收回時間

    private static final double GATE_CLOSED = 0.0;
    private static final double GATE_L_OPEN = 0.6667;
    private static final double GATE_R_OPEN = 0.6902;

    private static final float SENSOR_GAIN = 25.0f;
    private static final float MIN_DETECT_BRIGHTNESS = 0.7f;
    private static final float PURPLE_RATIO_LIMIT = 1.2f;
    private static final double INTAKE_POWER = -1.0;

    // === 狀態機定義 ===

    // 裝球狀態機
    private enum FillState {
        IDLE,           // 等待球
        WAIT_SETTLE,    // 球剛進入，等待穩定
        ROTATING,       // 圓盤轉動中
        FULL            // 已滿
    }

    // 發射狀態機
    private enum FireState {
        IDLE,           // 閒置
        PREPARING,      // 關閘門，發射輪加速
        DECIDING,       // 決定下一個要射哪個洞
        AIMING,         // 圓盤轉向目標洞
        KICKING,        // 長棍踢出
        RETRACTING,     // 長棍收回
        RESETTING       // 發射完畢，復位
    }

    // === 運行時變量 ===
    private FillState fillState = FillState.IDLE;
    private FireState fireState = FireState.IDLE;

    private long fillTimer = 0;
    private long fireTimer = 0;

    private int currentFillStep = 0;
    private boolean hasBallA = false, hasBallB = false, hasBallC = false;
    private String colorHoleA = "EMPTY", colorHoleB = "EMPTY", colorHoleC = "EMPTY";

    // 臨時變量，用於發射邏輯
    private double targetFirePos = 0;
    private String currentTargetHole = "";

    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Non-Blocking Initialized");
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


            // 1. 發射觸發 (只有在閒置時才能觸發)
            if (gamepad1.left_bumper && fireState == FireState.IDLE) {
                if (hasBallA || hasBallB || hasBallC) {
                    // 開始發射流程
                    fireState = FireState.PREPARING;
                    fireTimer = System.currentTimeMillis();

                    // 執行初始化動作
                    controlGates(false); // 關閘
                    shooterMotor.setPower(0.8); // 開發射輪
                }
            }

            // 2. 執行狀態機邏輯
            runFiringLogic();   // 處理發射
            runFillingLogic();  // 處理裝球
            runIntakeLogic();   // 處理進球馬達

            // 3. 顯示資訊
            updateTelemetry();
        }
    }

    // === 狀態機邏輯：裝球 ===
    private void runFillingLogic() {
        // 如果正在發射，暫停裝球邏輯
        if (fireState != FireState.IDLE) return;

        // 如果滿了，狀態設為 FULL
        if (currentFillStep >= 3) {
            fillState = FillState.FULL;
            return;
        } else if (fillState == FillState.FULL) {
            // 如果從滿變不滿 (例如剛發射完)，重置為 IDLE
            fillState = FillState.IDLE;
        }

        switch (fillState) {
            case IDLE:
                // 檢測顏色
                DetectedColor detectedColor = getDetectedColor(colorSensor);
                if (detectedColor != DetectedColor.UNKNOWN) {
                    // 記錄顏色
                    recordBallColor(detectedColor);

                    // 開始等待球穩定
                    fillTimer = System.currentTimeMillis();
                    fillState = FillState.WAIT_SETTLE;
                }
                break;

            case WAIT_SETTLE:
                // 等待球完全掉入 (TIME_BALL_SETTLE)
                if (System.currentTimeMillis() - fillTimer > TIME_BALL_SETTLE) {
                    // 轉動到下一個位置
                    moveToNextFillPosition();

                    // 開始轉動計時
                    fillTimer = System.currentTimeMillis();
                    fillState = FillState.ROTATING;
                }
                break;

            case ROTATING:
                // 等待圓盤轉動完成 (TIME_DISK_MOVE)
                if (System.currentTimeMillis() - fillTimer > TIME_DISK_MOVE) {
                    // 轉動完成，回到 IDLE 等待下一顆球
                    fillState = FillState.IDLE;
                }
                break;

            case FULL:
                // 什麼都不做，等待發射清空
                break;
        }
    }

    // === 狀態機邏輯：發射 ===
    private void runFiringLogic() {
        switch (fireState) {
            case IDLE:
                // 什麼都不做
                break;

            case PREPARING:
                // 等待發射輪加速 (TIME_SHOOTER_SPIN)
                if (System.currentTimeMillis() - fireTimer > TIME_SHOOTER_SPIN) {
                    fireState = FireState.DECIDING;
                }
                break;

            case DECIDING:
                // 判斷還有沒有球，並選擇最短路徑
                if (!hasBallA && !hasBallB && !hasBallC) {
                    // 沒球了，結束
                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.RESETTING;

                    // 執行復位動作
                    shooterMotor.setPower(0.0);
                    diskServo.setPosition(FILL_POS_STEP_1);
                } else {
                    // 還有球，選擇目標
                    selectBestTarget();

                    // 執行瞄準動作
                    diskServo.setPosition(targetFirePos);

                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.AIMING;
                }
                break;

            case AIMING:
                // 等待圓盤轉到發射位 (TIME_DISK_MOVE)
                if (System.currentTimeMillis() - fireTimer > TIME_DISK_MOVE) {
                    // 執行踢球
                    kickerServo.setPosition(KICKER_EXTEND);

                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.KICKING;
                }
                break;

            case KICKING:
                // 等待踢出 (TIME_KICK_OUT)
                if (System.currentTimeMillis() - fireTimer > TIME_KICK_OUT) {
                    // 執行收回
                    kickerServo.setPosition(KICKER_REST);

                    // 清除該洞的球狀態
                    clearBallStatus(currentTargetHole);

                    fireTimer = System.currentTimeMillis();
                    fireState = FireState.RETRACTING;
                }
                break;

            case RETRACTING:
                // 等待收回 (TIME_KICK_RETRACT)
                if (System.currentTimeMillis() - fireTimer > TIME_KICK_RETRACT) {
                    // 這一輪射擊完成，回到 DECIDING 判斷下一顆
                    fireState = FireState.DECIDING;
                }
                break;

            case RESETTING:
                // 等待圓盤復位 (稍作等待確保圓盤轉回去)
                if (System.currentTimeMillis() - fireTimer > 600) {
                    controlGates(true); // 開閘
                    currentFillStep = 0; // 重置計數
                    fireState = FireState.IDLE;
                }
                break;
        }
    }

    // === 進球馬達邏輯 ===
    private void runIntakeLogic() {
        // 只有在 (沒滿) 且 (不在發射狀態) 時開啟吸球
        // 注意：fireState != IDLE 代表正在發射中
        if (currentFillStep < 3 && fireState == FireState.IDLE) {
            intakeMotor.setPower(INTAKE_POWER);
        } else {
            intakeMotor.setPower(0.0);
        }
    }

    // === 輔助方法 ===

    private void recordBallColor(DetectedColor color) {
        switch (currentFillStep) {
            case 0:
                colorHoleA = color.toString();
                hasBallA = true;
                break;
            case 1:
                colorHoleB = color.toString();
                hasBallB = true;
                break;
            case 2:
                colorHoleC = color.toString();
                hasBallC = true;
                break;
        }
    }

    private void moveToNextFillPosition() {
        if (currentFillStep == 0) {
            diskServo.setPosition(FILL_POS_STEP_2);
            currentFillStep = 1;
        } else if (currentFillStep == 1) {
            diskServo.setPosition(FILL_POS_STEP_3);
            currentFillStep = 2;
        } else if (currentFillStep == 2) {
            // 滿了，不轉動
            currentFillStep = 3;
        }
    }

    private void selectBestTarget() {
        double currentPos = diskServo.getPosition();
        double distA = hasBallA ? Math.abs(currentPos - FIRE_POS_HOLE_A) : 999.0;
        double distB = hasBallB ? Math.abs(currentPos - FIRE_POS_HOLE_B) : 999.0;
        double distC = hasBallC ? Math.abs(currentPos - FIRE_POS_HOLE_C) : 999.0;

        if (distA <= distB && distA <= distC) {
            targetFirePos = FIRE_POS_HOLE_A;
            currentTargetHole = "A";
        } else if (distB <= distA && distB <= distC) {
            targetFirePos = FIRE_POS_HOLE_B;
            currentTargetHole = "B";
        } else {
            targetFirePos = FIRE_POS_HOLE_C;
            currentTargetHole = "C";
        }
    }

    private void clearBallStatus(String hole) {
        if (hole.equals("A")) { hasBallA = false; colorHoleA = "EMPTY"; }
        if (hole.equals("B")) { hasBallB = false; colorHoleB = "EMPTY"; }
        if (hole.equals("C")) { hasBallC = false; colorHoleC = "EMPTY"; }
    }

    private void controlGates(boolean isOpen) {
        if (isOpen) {
            gateServoL.setPosition(GATE_L_OPEN);
            gateServoR.setPosition(GATE_R_OPEN);
        } else {
            gateServoL.setPosition(GATE_CLOSED);
            gateServoR.setPosition(GATE_CLOSED);
        }
    }

    private void initHardware() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        if (colorSensor instanceof com.qualcomm.robotcore.hardware.SwitchableLight) {
            ((com.qualcomm.robotcore.hardware.SwitchableLight)colorSensor).enableLight(true);
        }
        colorSensor.setGain(SENSOR_GAIN);

        kickerServo = hardwareMap.get(Servo.class, "servo1");
        diskServo = hardwareMap.get(Servo.class, "servo2");
        gateServoL = hardwareMap.get(Servo.class, "servo4");
        gateServoR = hardwareMap.get(Servo.class, "servo5");

        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
        shooterMotor = hardwareMap.get(DcMotor.class, "motor5");
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

        // 依照之前的設定
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
        telemetry.addLine("=== SYSTEM STATUS ===");
        telemetry.addData("Fill State", fillState);
        telemetry.addData("Fire State", fireState);

        if (fireState != FireState.IDLE) {
            telemetry.addData("Action", "FIRING in progress...");
        } else {
            telemetry.addData("Action", "Intake / Idle");
        }

        telemetry.addLine("\n=== BALLS ===");
        telemetry.addData("A", "[%s] %s", colorHoleA, hasBallA?"●":"○");
        telemetry.addData("B", "[%s] %s", colorHoleB, hasBallB?"●":"○");
        telemetry.addData("C", "[%s] %s", colorHoleC, hasBallC?"●":"○");

        telemetry.update();
    }
}