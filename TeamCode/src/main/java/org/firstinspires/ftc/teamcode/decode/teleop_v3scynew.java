package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop_ken")
public class teleop_v3scynew extends LinearOpMode {

    // === 1. 硬體組件 ===
    private Limelight3A limelight;
    private NormalizedColorSensor colorSensor1;
    private Servo kickerServo, gateServoL, gateServoR, angleServo;
    private CRServo diskServo;
    private DcMotor intakeMotor, baseMotor;
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private DcMotorEx shooterMotorLeft, shooterMotorRight;
    private LED LED0, LED1, LED2;

    // === 2. 針對 -700~800 抖動優化的 PID 參數 ===
    private double diskTargetPosition = 0;
    private double diskIntegral = 0;
    private double diskLastError = 0;

    // === 針對 40:10 高速低扭力優化的 PID 參數 ===

    // 遠程加速：稍微降低，避免衝得太狂暴導致皮帶/齒輪掃齒
    private static final double K_P_FAST = 0.0015;

    // 近程制動：極小的 P，確保不會在洞口附近反覆震盪
    private static final double K_P_STABLE = 0.0001;

    // 極大的 D (阻尼)：這是 40:10 系統的靈魂！
    // 就像開超跑必須配備碳纖維陶瓷煞車卡鉗一樣，數值需要比以前大很多
    private static final double K_D_STABLE = 0.0045;

    private static final double K_I_STABLE = 0.00000;
    // 以前是 110，現在必須縮小，否則物理對位會差非常多！
    // 建議先從 30 開始測試，如果會卡死抖動再微調到 40 或 50
    private static final double DISK_TOLERANCE = 30;   // 寬死區是加速比結構的救星
    private static final double DISK_MAX_POWER = 1.0;

    // === 3. 位置設定 ===
    private static final double POS_FILL_A = 0;
    private static final double POS_FILL_B = 2706;
    private static final double POS_FILL_C = 5483;
    private static final double POS_FIRE_A = 6200;
    private static final double POS_FIRE_B = 8964;
    private static final double POS_FIRE_C = 11683;

    // === 4. 狀態機變數 ===
    private enum FillState { IDLE, WAIT_SETTLE, ROTATING }
    private enum FireState { IDLE, PREPARING, DECIDING, AIMING, KICKING, RETRACTING, RESETTING }
    private FillState fillState = FillState.IDLE;
    private FireState fireState = FireState.IDLE;
    private int currentFillStep = 0;
    private boolean hasBallA = false, hasBallB = false, hasBallC = false;
    private String currentTargetHole = "NONE";
    private long fillTimer = 0, fireTimer = 0;
    private double lastDiskPower = 0;
    private boolean lastButtonA = false;

    @Override
    public void runOpMode() {
        initHardware();
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        diskTargetPosition = POS_FILL_A;

        waitForStart();

        while (opModeIsActive()) {
            updateDiskPID();
            handleDriveAndLimelight();
            handleShooterLogic();
            runFillingLogic();
            runFiringLogic();

            // 1. Intake 長期 Power On //CKH
            // === 替換原本的 Intake 控制 ===

            if (gamepad1.y) {
                // 手動強制退彈 (玩家按下 Y 鍵)
                intakeMotor.setPower(-1.0);
            } else {
                // 自動 Intake 邏輯
                if (fillState == FillState.WAIT_SETTLE || fillState == FillState.ROTATING) {
                    // 【防卡死核心】：當轉盤正在轉動換洞，或是剛吃到球還沒穩定時
                    // 讓 Intake 微微反轉 (-0.3 到 -0.4)，把緊跟在後面的第二顆球「擋在門外」！
                    intakeMotor.setPower(-0.1);
                }
                else if (currentFillStep >= 3 && fireState == FireState.IDLE) {
                    // 【防溢出】：當已經吸滿 3 顆球，且還沒開始發射時，徹底停止 Intake 避免吃進第 4 顆
                    intakeMotor.setPower(0.0);
                }
                else {
                    // 轉盤已經準備好 (IDLE)，且洞口已完美對準，火力全開吸球！
                    intakeMotor.setPower(1.0);
                }
            }

            // 2. 手動 A 鍵退回
            if (gamepad1.a && !lastButtonA) handleManualRetract();
            lastButtonA = gamepad1.a;

            updateTelemetry();
        }
    }

    private void updateDiskPID() {
        // 當吸滿 3 個 artifacts 且不在射擊時，徹底停轉
        if (currentFillStep >= 3 && fireState == FireState.IDLE) {
            diskServo.setPower(0);
            lastDiskPower = 0;
            return;
        }

        double currentPos = intakeMotor.getCurrentPosition();
        double error = diskTargetPosition - currentPos;
        double absError = Math.abs(error);
        double derivative = error - diskLastError;

        // 【新增邏輯】：動態決定死區 (Tolerance)
        // 進球時(IDLE)死區放大到 200，射擊時保持精準的 110
        double currentTolerance = (fireState == FireState.IDLE) ? 200 : DISK_TOLERANCE;

        // 到位鎖死邏輯
        if (absError < currentTolerance) {
            diskServo.setPower(0);
            diskIntegral = 0;
            lastDiskPower = 0;
            diskLastError = error;
            return;
        }

        // 【核心提速邏輯】：動態決定煞車距離
        // 進球時(IDLE)縮短煞車距離到 800 (晚點煞車，保持高速更久)
        // 射擊時提早到 1800 煞車，確保平穩不卡死
        double brakingDistance = (fireState == FireState.IDLE) ? 800 : 1800;

        double pOutput, dOutput;
        if (absError < brakingDistance) {
            // 進入煞車區
            pOutput = error * K_P_STABLE;
            dOutput = derivative * K_D_STABLE;
        } else {
            // 遠程狂飆區
            pOutput = error * K_P_FAST;
            dOutput = derivative * K_D_STABLE * 0.5; // 高速時不需要這麼強的阻尼
        }

        // 接近時才啟用積分以修正 PLA 齒輪靜摩擦
        if (absError < 400) diskIntegral += error;
        else diskIntegral = 0;
        double iOutput = diskIntegral * K_I_STABLE;

        double power = pOutput + iOutput + dOutput;

        // 最小功率門檻（防止低頻震盪）
        if (Math.abs(power) < 0.05) power = 0;

        lastDiskPower = Math.max(-DISK_MAX_POWER, Math.min(DISK_MAX_POWER, power));
        diskServo.setPower(lastDiskPower);
        diskLastError = error;
    }
    private void runFillingLogic() {
        if (currentFillStep >= 3 || fireState != FireState.IDLE) return;

        NormalizedRGBA c = colorSensor1.getNormalizedColors();
        if (fillState == FillState.IDLE && c.alpha > 0.75) {
            if (currentFillStep == 0) { hasBallA = true; LED0.on(); }
            else if (currentFillStep == 1) { hasBallB = true; LED1.on(); }
            else if (currentFillStep == 2) { hasBallC = true; LED2.on(); }
            fillTimer = System.currentTimeMillis();
            fillState = FillState.WAIT_SETTLE;
        } else if (fillState == FillState.WAIT_SETTLE && System.currentTimeMillis() - fillTimer > 200) {
            moveToNextFillPosition();
            fillTimer = System.currentTimeMillis();
            fillState = FillState.ROTATING;
        } else if (fillState == FillState.ROTATING && System.currentTimeMillis() - fillTimer > 400) {
            fillState = FillState.IDLE;
        }
    }

    private void runFiringLogic() {
        switch (fireState) {
            case IDLE:
                if (gamepad1.right_trigger > 0.1) {
                    fireTimer = System.currentTimeMillis(); fireState = FireState.PREPARING; controlGates(false);
                } break;
            case PREPARING:
                if (System.currentTimeMillis() - fireTimer > 600) fireState = FireState.DECIDING; break;
            case DECIDING:
                if (hasBallC) { diskTargetPosition = POS_FIRE_C; currentTargetHole = "C"; switchToAiming(); LED2.off(); }
                else if (hasBallB) { diskTargetPosition = POS_FIRE_B; currentTargetHole = "B"; switchToAiming(); LED1.off(); }
                else if (hasBallA) { diskTargetPosition = POS_FIRE_A; currentTargetHole = "A"; switchToAiming(); LED0.off(); }
                else { fireState = FireState.RESETTING; fireTimer = System.currentTimeMillis(); diskTargetPosition = POS_FILL_A; }
                break;
            case AIMING://CKH
                double currentError = Math.abs(diskTargetPosition - intakeMotor.getCurrentPosition());

                // 【核心修正】：
                // 數學上，當誤差小於 500 時，馬達就會因為推力不足 0.15 而斷電。
                // 既然視覺上這個位置已經對準，我們就把開火的容許誤差放寬到 500。
                // 只要誤差 < 500，且馬達已經放棄出力 (lastDiskPower == 0)，就視為「瞄準完成」！
                if (currentError <= 500 && lastDiskPower == 0) {

                    // 確保轉盤在這個狀態下「完美靜止」維持了 200 毫秒才踢，防止剛好滑過去被誤判
                    if (System.currentTimeMillis() - fireTimer > 200) {
                        kickerServo.setPosition(0.8);
                        fireTimer = System.currentTimeMillis();
                        fireState = FireState.KICKING;
                    }
                } else {
                    // 如果馬達還在出力微調 (lastDiskPower != 0)，或者誤差實在太大
                    // 就重置計時器，繼續等它停穩
                    fireTimer = System.currentTimeMillis();
                }
                break;
            case KICKING:
                if (System.currentTimeMillis() - fireTimer > 150) {
                    kickerServo.setPosition(0.0); clearBallStatus(currentTargetHole);
                    fireTimer = System.currentTimeMillis(); fireState = FireState.RETRACTING;
                } break;
            case RETRACTING:
                if (System.currentTimeMillis() - fireTimer > 150) fireState = FireState.DECIDING; break;
            case RESETTING:
                if (System.currentTimeMillis() - fireTimer > 400) {
                    controlGates(true); currentFillStep = 0; fireState = FireState.IDLE;
                } break;
        }
    }

    private void initHardware() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        kickerServo = hardwareMap.get(Servo.class, "servo1");
        diskServo = hardwareMap.get(CRServo.class, "servo2");
        gateServoL = hardwareMap.get(Servo.class, "servo4");
        gateServoR = hardwareMap.get(Servo.class, "servo5");
        angleServo = hardwareMap.get(Servo.class, "servo3");
        angleServo.setDirection(Servo.Direction.REVERSE);
        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "motor5");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "motor7");
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
        baseMotor = hardwareMap.get(DcMotor.class, "motor6");
        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LED0 = hardwareMap.get(LED.class, "LED0");
        LED1 = hardwareMap.get(LED.class, "LED1");
        LED2 = hardwareMap.get(LED.class, "LED2");
        kickerServo.scaleRange(0, 0.5);
        kickerServo.setPosition(0);
        controlGates(true);
    }

    private void handleManualRetract() {
        if (currentFillStep > 0) {
            currentFillStep--;
            diskTargetPosition = (currentFillStep==0?POS_FILL_A : (currentFillStep==1?POS_FILL_B:POS_FILL_C));
        }
    }

    private void handleDriveAndLimelight() {
        double x = gamepad1.left_stick_x, y = -gamepad1.left_stick_y, rx = gamepad1.right_stick_x;
        frontLeftMotor.setPower(y+x+rx); frontRightMotor.setPower(y-x-rx);
        backLeftMotor.setPower(y-x+rx); backRightMotor.setPower(y+x-rx);
    }

    private void handleShooterLogic() {
        shooterMotorRight.setVelocity(gamepad1.right_trigger > 0.1 ? 2200 : 300);
        shooterMotorLeft.setPower(shooterMotorRight.getPower());
    }

    private void moveToNextFillPosition() {
        if (currentFillStep == 0) { diskTargetPosition = POS_FILL_B; currentFillStep = 1; }
        else if (currentFillStep == 1) { diskTargetPosition = POS_FILL_C; currentFillStep = 2; }
        else if (currentFillStep == 2) { currentFillStep = 3; }
    }

    private void controlGates(boolean o) { gateServoL.setPosition(o ? 0.6667 : 0.32); gateServoR.setPosition(o ? 0.6902 : 0.32); }
    private void clearBallStatus(String h) { if(h.equals("A")) hasBallA=false; if(h.equals("B")) hasBallB=false; if(h.equals("C")) hasBallC=false; }
    private void switchToAiming() { fireTimer = System.currentTimeMillis(); fireState = FireState.AIMING; }

    private void updateTelemetry() {
        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Artifact Count", currentFillStep);
        telemetry.addData("Intake", "CONSTANT ON");
        telemetry.addLine("--- PID DEBUG ---");
        telemetry.addData("Error", (int)(diskTargetPosition - intakeMotor.getCurrentPosition()));
        telemetry.addData("Disk Power", String.format("%.2f", lastDiskPower));
        telemetry.addData("Mode", Math.abs(diskTargetPosition - intakeMotor.getCurrentPosition()) < 1000 ? "STABLE" : "FAST");
        telemetry.update();
    }
}