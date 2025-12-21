package org.firstinspires.ftc.teamcode.decode;

import android.graphics.Color; // 必須導入這行來使用 HSV
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight; // 用於控制燈光
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "teleop_v2")
public class teleop_v2 extends LinearOpMode {

    // 顏色感測器
    NormalizedColorSensor intakeColorSensor;

    // Servos & Motors (保持原樣)
    Servo rotationBaseServo;
    Servo firingServo;
    CRServo angleServo;
    Servo zhaServo4;
    Servo zhaServo5;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    DcMotor intakeMotor4, shooterMotor;

    // 位置常數 (保持原樣)
    private static final double[] BASE_POSITIONS = {0.0, 0.4, 0.7255};
    private static final double[] INTAKE_POSITIONS = {0.0, 0.37, 0.73};
    private static final double FIRING_SERVO_ACTIVE = 0.7;
    private static final double FIRING_SERVO_REST = 0.0;
    private static final double ZHA_SERVO_POSITION_OPEN = 1.0;
    private static final double ZHA_SERVO_POSITION_CLOSE = 0.0;

    public enum BallColor { UNKNOWN, PURPLE, GREEN }

    private class BallSlot {
        public BallColor color = BallColor.UNKNOWN;
        public boolean hasBall = false;
        public int position;
    }

    private BallSlot[] ballSlots = new BallSlot[3];

    // 控制變數 (保持原樣)
    private boolean sequenceInProgress = false;
    private boolean intakeMode = false;
    private int currentBasePosition = 0;
    private int nextIntakePosition = 0;
    private boolean settingSequence = false;
    private BallColor[] manualSequence = new BallColor[3];
    private int sequenceStep = 0;
    private boolean zhaServoOpen = false;

    // 按鍵防抖
    private boolean lastLeftBumper = false, lastRightBumper = false;
    private boolean lastAButton = false, lastBButton = false;
    private boolean lastXButton = false, lastYButton = false;
    private boolean lastDpadUp = false, lastDpadDown = false;
    private boolean lastDpadLeft = false, lastDpadRight = false;

    // *** 顏色檢測變數 (新增 HSV 相關) ***
    private float[] hsvValues = new float[3]; // hsvValues[0]=Hue, [1]=Sat, [2]=Val
    private BallColor lastDetectedColor = BallColor.UNKNOWN;
    private long lastDetectionTime = 0;
    private static final long DETECTION_COOLDOWN_MS = 500;

    @Override
    public void runOpMode() {
        initHardware();

        // 初始化變數
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = new BallSlot();
            ballSlots[i].position = i;
        }
        resetManualSequence();

        telemetry.addData("狀態", "系統就緒 (HSV模式)");
        telemetry.addData("提示", "若讀數仍為0，請檢查接線與Config名稱");
        telemetry.update();

        waitForStart();

        rotationBaseServo.setPosition(BASE_POSITIONS[0]);

        while (opModeIsActive()) {
            driveRobot();
            handleManualSequenceSetting();
            handleShooterControls();
            handleModeSwitching();
            handleIntakeControl();
            handleAutoFiring();
            handleManualFiring();
            handleZhaServoControl();
            handleIntakePositionControl();
            updateTelemetry();
        }
        resetAllServos();

    }

    private void initHardware() {
        // *** 顏色感測器重要修復 ***
        try {
            intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");

            // 1. 強制開啟補光燈 (如果不開燈，在機器內部會讀到全黑/0)
            if (intakeColorSensor instanceof SwitchableLight) {
                ((SwitchableLight) intakeColorSensor).enableLight(true);
            }

            // 2. 設定增益 (Gain)
            // 預設是 1.0，如果環境暗，讀數會很小。設為 15-25 可以顯著放大讀數。
            intakeColorSensor.setGain(20);

        } catch (Exception e) {
            telemetry.addData("錯誤", "找不到 colorSensor0，請檢查 Config");
            telemetry.update();
        }

        // 初始化其他硬體 (保持原樣)
        rotationBaseServo = hardwareMap.get(Servo.class, "servo2");
        firingServo = hardwareMap.get(Servo.class, "servo1");
        angleServo = hardwareMap.get(CRServo.class, "servo3");
        zhaServo4 = hardwareMap.get(Servo.class, "servo4");
        zhaServo5 = hardwareMap.get(Servo.class, "servo5");
        firingServo.scaleRange(0, 0.5);

        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
        intakeMotor4 = hardwareMap.get(DcMotor.class, "motor4");
        shooterMotor = hardwareMap.get(DcMotor.class, "motor5");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        zhaServo4.setDirection(Servo.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetAllServos();
    }

    // (省略: resetAllServos, driveRobot, handleManualSequenceSetting, handleShooterControls, handleModeSwitching... 這些與原程式相同)
    // 為了節省篇幅，請保留你原有的這些方法。

    // --- 必須保留的方法佔位符 (請複製你原本的內容) ---
    private void resetAllServos() { firingServo.setPosition(FIRING_SERVO_REST); angleServo.setPower(0); sleep(300); }
    private void driveRobot() { /* 複製原本的 */
        double x = gamepad1.left_stick_x; double y = -gamepad1.left_stick_y; double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);
    }
    private void handleManualSequenceSetting() { /* 複製原本的 */
        if(gamepad1.right_bumper && !lastRightBumper) { settingSequence=true; sequenceStep=0; }
        if(!gamepad1.right_bumper && lastRightBumper && settingSequence) { settingSequence=false; }
        lastRightBumper = gamepad1.right_bumper;
        if(settingSequence && sequenceStep<3) {
            if(gamepad1.a && !lastAButton) { manualSequence[sequenceStep++] = BallColor.GREEN; }
            if(gamepad1.b && !lastBButton) { manualSequence[sequenceStep++] = BallColor.PURPLE; }
            lastAButton=gamepad1.a; lastBButton=gamepad1.b;
        }
    }
    private void handleShooterControls() { shooterMotor.setPower(gamepad1.right_trigger); angleServo.setPower(gamepad1.right_stick_x * 0.5); }
    private void handleModeSwitching() { /* 複製原本的 */
        if(gamepad1.y && !lastYButton && !sequenceInProgress) {
            intakeMode = !intakeMode;
            rotationBaseServo.setPosition(intakeMode ? INTAKE_POSITIONS[0] : BASE_POSITIONS[0]);
            currentBasePosition = 0;
        }
        lastYButton = gamepad1.y;
    }

    // *** 修改過的 Intake Control (使用新的檢測邏輯) ***
    private void handleIntakeControl() {
        boolean currentDpadLeft = gamepad1.dpad_left;
        boolean currentDpadRight = gamepad1.dpad_right;

        if (currentDpadLeft && !lastDpadLeft) {
            intakeMotor4.setPower(1);
            if (intakeMode) {
                long currentTime = System.currentTimeMillis();
                if (currentTime - lastDetectionTime > DETECTION_COOLDOWN_MS) {
                    // 使用新的檢測方法
                    BallColor detectedColor = checkColorHSV();

                    if (detectedColor != BallColor.UNKNOWN) {
                        lastDetectionTime = currentTime;
                        // 存入球並轉動
                        ballSlots[currentBasePosition].hasBall = true;
                        ballSlots[currentBasePosition].color = detectedColor;
                        moveToNextIntakePosition();
                    }
                }
            }
        } else if (currentDpadRight && !lastDpadRight) {
            intakeMotor4.setPower(0);
        }
        lastDpadLeft = currentDpadLeft;
        lastDpadRight = currentDpadRight;
    }

    // (省略: moveToNextIntakePosition, handleAutoFiring... 請保留原本內容)
    private void moveToNextIntakePosition() { /* 複製原本的 */
        nextIntakePosition = (currentBasePosition + 1) % 3;
        int attempts = 0;
        while (ballSlots[nextIntakePosition].hasBall && attempts < 3) { nextIntakePosition = (nextIntakePosition + 1) % 3; attempts++; }
        if (!ballSlots[nextIntakePosition].hasBall) {
            currentBasePosition = nextIntakePosition;
            rotationBaseServo.setPosition(INTAKE_POSITIONS[currentBasePosition]);
        }
    }
    private void handleAutoFiring() { /* 複製原本的 */ if(gamepad1.left_bumper && !lastLeftBumper && !sequenceInProgress) { sequenceInProgress=true; new Thread(()->{executeFiringSequence(); sequenceInProgress=false;}).start(); } lastLeftBumper=gamepad1.left_bumper; }
    private void handleManualFiring() { if(gamepad1.left_bumper && !lastLeftBumper && !sequenceInProgress) fireAtCurrentPosition(); }
    private void handleZhaServoControl() { /* 複製原本的 */ if(gamepad1.dpad_up) {zhaServo4.setPosition(1); zhaServo5.setPosition(1); zhaServoOpen=true;} if(gamepad1.dpad_down) {zhaServo4.setPosition(0); zhaServo5.setPosition(0); zhaServoOpen=false;} }
    private void handleIntakePositionControl() { /* 複製原本的 */ if(intakeMode){ if(gamepad1.a) {currentBasePosition=0; rotationBaseServo.setPosition(INTAKE_POSITIONS[0]);} if(gamepad1.b) {currentBasePosition=1; rotationBaseServo.setPosition(INTAKE_POSITIONS[1]);} if(gamepad1.x) {currentBasePosition=2; rotationBaseServo.setPosition(INTAKE_POSITIONS[2]);} } }
    private void fireAtCurrentPosition() { /* 複製原本的 */ firingServo.setPosition(FIRING_SERVO_ACTIVE); sleep(300); firingServo.setPosition(FIRING_SERVO_REST); ballSlots[currentBasePosition].hasBall=false; ballSlots[currentBasePosition].color=BallColor.UNKNOWN; }
    private void executeFiringSequence() { /* 複製原本的，確保調用新的 hasValidManualSequence 等方法 */
        intakeMode = false; currentBasePosition=0; rotationBaseServo.setPosition(BASE_POSITIONS[0]); sleep(500);
        int[] firingOrder = hasValidManualSequence() ? getFiringOrderFromManualSequence() : getReverseFiringOrder();
        for(int pos : firingOrder) {
            if(pos != -1) {
                currentBasePosition = pos; rotationBaseServo.setPosition(BASE_POSITIONS[pos]); sleep(500);
                firingServo.setPosition(FIRING_SERVO_ACTIVE); sleep(300); firingServo.setPosition(FIRING_SERVO_REST);
                ballSlots[pos].hasBall=false; ballSlots[pos].color=BallColor.UNKNOWN;
            }
        }
        currentBasePosition=0; rotationBaseServo.setPosition(BASE_POSITIONS[0]);
    }
    private boolean hasValidManualSequence() { for(BallColor c:manualSequence) if(c!=BallColor.UNKNOWN) return true; return false; }
    private int[] getFiringOrderFromManualSequence() { /* 複製原本的 */ int[] order={-1,-1,-1}; boolean[] used=new boolean[3]; int idx=0; for(BallColor c:manualSequence){ if(c!=BallColor.UNKNOWN) for(int j=0;j<3;j++) if(!used[j] && ballSlots[j].hasBall && ballSlots[j].color==c){ order[idx++]=j; used[j]=true; break;} } for(int j=2;j>=0;j--) if(!used[j] && ballSlots[j].hasBall) order[idx++]=j; return order; }
    private int[] getReverseFiringOrder() { /* 複製原本的 */ int[] order={-1,-1,-1}; int idx=0; for(int i=2;i>=0;i--) if(ballSlots[i].hasBall) order[idx++]=i; return order; }
    private void resetManualSequence() { for(int i=0;i<3;i++) manualSequence[i]=BallColor.UNKNOWN; sequenceStep=0; }

    // *** 新增：使用 HSV 進行穩定的顏色判斷 ***
    // *** 根據實測數據修正後的判斷邏輯 ***
    private BallColor checkColorHSV() {
        NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        float hue = hsvValues[0];
        float sat = hsvValues[1];
        float val = hsvValues[2];

        // 1. 亮度檢查 (太暗 = 無球)
        if (val < 0.15) {
            return BallColor.UNKNOWN;
        }

        // 2. 顏色判斷 (基於你的實測數據)
        // 綠色: Hue~160, Sat~0.61 (高飽和)
        // 紫色: Hue~174, Sat~0.44 (低飽和)

        if (hue > 140 && hue <= 167 && sat > 0.50) {
            return BallColor.GREEN;
        }
        else if (hue > 167 && hue < 200 && sat < 0.55) {
            return BallColor.PURPLE;
        }

        return BallColor.UNKNOWN;
    }

    // 更新 Telemetry (微調)
    private void updateTelemetry() {
        telemetry.clear();

        // --- 第一區：即時判定 (字體最大最清楚) ---
        telemetry.addLine("=== 👁️ 視覺感測判定 ===");

        // 再次呼叫獲取最新 HSV (僅用於顯示)
        checkColorHSV();
        BallColor currentColor = lastDetectedColor; // 或者是 checkColorHSV() 的結果

        String statusSymbol = "⬛"; // 預設無球
        String statusText = "等待中...";

        // 判斷目前看到什麼，根據 hsvValues 顯示
        // 注意：這裡直接拿 checkColorHSV 更新過的 hsvValues 來判斷顯示文字
        float h = hsvValues[0];
        float s = hsvValues[1];
        float v = hsvValues[2];

        if (v < 0.15) {
            statusSymbol = "⬛ (空)";
            statusText = "太暗 / 無物體";
        } else if (h > 140 && h <= 167 && s > 0.50) {
            statusSymbol = "🟢 綠色";
            statusText = "高飽和 (Sat > 0.5)";
        } else if (h > 167 && h < 200 && s < 0.55) {
            statusSymbol = "🟣 紫色";
            statusText = "低飽和 (Sat < 0.55)";
        } else {
            statusSymbol = "⚠️ 未知";
            statusText = "數值模糊地帶";
        }

        telemetry.addData("目前看到", "%s", statusSymbol);
        telemetry.addData("判定依據", statusText);

//        telemetry.addLine("\n--- 📊 詳細數據 (HSV) ---");
//        // 使用圖形條顯示數值，一眼就能看出高低
//        telemetry.addData("色相 (Hue)", "%.0f (綠<167<紫)", h);
//        telemetry.addData("飽和 (Sat)", "%.2f %s", s, getProgressBar(s));
//        telemetry.addData("亮度 (Val)", "%.2f %s", v, getProgressBar(v));
//        telemetry.addData("增益 (Gain)", "%d", intakeColorSensor.getGain());

        telemetry.addLine("\n--- 🛒 球槽狀態 ---");
        // 用視覺化括號顯示三個位置
        String slot1 = ballSlots[0].hasBall ? (ballSlots[0].color == BallColor.GREEN ? "🟢" : "🟣") : "__";
        String slot2 = ballSlots[1].hasBall ? (ballSlots[1].color == BallColor.GREEN ? "🟢" : "🟣") : "__";
        String slot3 = ballSlots[2].hasBall ? (ballSlots[2].color == BallColor.GREEN ? "🟢" : "🟣") : "__";

        // 加上箭頭指示當前轉盤指向的位置
        String p1 = (currentBasePosition == 0) ? "^" : " ";
        String p2 = (currentBasePosition == 1) ? "^" : " ";
        String p3 = (currentBasePosition == 2) ? "^" : " ";

        telemetry.addData("儲存狀況", "[ %s ] [ %s ] [ %s ]", slot1, slot2, slot3);
        telemetry.addData("目前位置", "  %s      %s      %s  ", p1, p2, p3);

        // 放在 updateTelemetry 方法的下方，Class 結束大括號的上方


        telemetry.update();
    }
}