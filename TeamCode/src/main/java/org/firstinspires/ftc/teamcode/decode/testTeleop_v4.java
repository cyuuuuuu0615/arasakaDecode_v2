//package org.firstinspires.ftc.teamcode.decode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//@TeleOp(name = "testTeleop_v4")
//public class testTeleop_v4 extends LinearOpMode {
//
//    // Color Sensors (固定在旋轉底盤上)
//    NormalizedColorSensor colorSensor0;
//    NormalizedColorSensor colorSensor1;
//    NormalizedColorSensor colorSensor2;
//    NormalizedColorSensor colorSensor3;
//    NormalizedColorSensor colorSensor4;
//    NormalizedColorSensor colorSensor5;
//
//    // Servos
//    Servo rotationBaseServo;      // 旋轉底盤的servo
//    Servo firingServo;           // 發射servo (range 0-0.5)
//    CRServo angleServo;          // 發射角度調整的連續旋轉servo
//
//    // Motors
//    DcMotor frontLeftMotor;
//    DcMotor frontRightMotor;
//    DcMotor backLeftMotor;
//    DcMotor backRightMotor;
//    DcMotor intakeMotor4;
//    DcMotor shooterMotor;        // 發射馬達
//
//    // 位置常數
//    private static final double[] BASE_POSITIONS = {0.0, 0.4, 0.7255};     // 射擊位置
//    private static final double[] INTAKE_POSITIONS = {0.0, 0.37, 0.73};    // 進料位置
//    private static final double FIRING_SERVO_ACTIVE = 0.7;
//    private static final double FIRING_SERVO_REST = 0.0;
//
//    // 時間常數 (毫秒)
//    private static final int BASE_ROTATION_DELAY_MS = 500;    // 底盤轉動等待時間
//    private static final int FIRING_DELAY_MS = 500;           // 發射伺服器停留時間
//    private static final int SENSOR_READ_DELAY_MS = 100;      // 感測器讀取等待時間
//    private static final int FIRING_INTERVAL_MS = 1500;       // 發射間隔時間
//
//    // 模式控制
//    public enum DetectedColor {
//        PURPLE, GREEN, UNKNOWN
//    }
//
//    public enum ColorMode {
//        GREEN_PURPLE_PURPLE, PURPLE_GREEN_PURPLE, PURPLE_PURPLE_GREEN
//    }
//
//    private ColorMode currentColorMode = ColorMode.GREEN_PURPLE_PURPLE;
//    private boolean sequenceInProgress = false;
//    private boolean intakeMode = false;  // true=進料模式, false=射擊模式
//    private int currentBasePosition = 0; // 0,1,2 對應三個位置
//
//    @Override
//    public void runOpMode() {
//        initHardware();
//
//        telemetry.addData("狀態", "初始化完成 - 新設計版本");
//        telemetry.addData("設計特色", "單一旋轉底盤 + 單一發射機構");
//        telemetry.addData("底盤位置", "射擊: [0.0, 0.4, 0.7255], 進料: [0.0, 0.37, 0.73]");
//        telemetry.addData("控制按鈕", "A - 發射, B/X/Y - 切換模式");
//        telemetry.addData("控制按鈕", "左/右扳機 - 控制發射馬達功率");
//        telemetry.addData("控制按鈕", "左/右肩鍵 - 調整發射角度");
//        telemetry.addData("控制按鈕", "D-pad Up - 切換進料/射擊模式");
//        telemetry.addData("當前模式", currentColorMode);
//        telemetry.update();
//
//        waitForStart();
//
//        // 初始位置設置
//        rotationBaseServo.setPosition(BASE_POSITIONS[0]);
//        currentBasePosition = 0;
//        intakeMode = false;
//
//        while (opModeIsActive()) {
//            // 1. 處理車子移動
//            driveRobot();
//
//            // 2. 處理模式切換 (只在非序列進行中)
//            if (!sequenceInProgress) {
//                handleModeSelection();
//            }
//
//            // 3. 處理發射系統控制
//            handleShooterControls();
//
//            // 4. 處理進料/射擊模式切換
//            if (gamepad1.dpad_up && !sequenceInProgress) {
//                intakeMode = !intakeMode;
//                telemetry.addData("模式切換", intakeMode ? "進料模式" : "射擊模式");
//                updateBasePosition();
//                sleep(200);
//            }
//
//            // 5. 處理自動發射序列
//            if (gamepad1.a && !sequenceInProgress) {
//                sequenceInProgress = true;
//                new Thread(() -> {
//                    executeAutoFiringSequence();
//                    sequenceInProgress = false;
//                }).start();
//                sleep(200); // 防止重複觸發
//            }
//
//            // 6. 進料系統控制
//            if (gamepad1.dpad_left) {
//                intakeMotor4.setPower(1);
//            } else if (gamepad1.dpad_right) {
//                intakeMotor4.setPower(0);
//            }
//
//            // 更新telemetry
//            updateTelemetry();
//        }
//
//    }
//
//    private void initHardware() {
//        // 初始化顏色感測器
//        colorSensor0 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");
//        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
//        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
//        colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor3");
//        colorSensor4 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor4");
//        colorSensor5 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor5");
//
//        // 初始化Servos
//        rotationBaseServo = hardwareMap.get(Servo.class, "servo2");
//        firingServo = hardwareMap.get(Servo.class, "servo1");
//        angleServo = hardwareMap.get(CRServo.class, "servo3");
//
//        // 設置servo範圍
//        firingServo.scaleRange(0, 0.5);
//
//        // 初始化馬達
//        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
//        frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
//        backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
//        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
//        intakeMotor4 = hardwareMap.get(DcMotor.class, "motor4");
//        shooterMotor = hardwareMap.get(DcMotor.class, "motor5");
//
//        // 設置馬達方向
//        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        // 設置煞車模式
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // 重置servo位置
//        firingServo.setPosition(FIRING_SERVO_REST);
//        angleServo.setPower(0);
//    }
//
//    private void driveRobot() {
//        double x = gamepad1.left_stick_x;
//        double y = -gamepad1.left_stick_y;
//        double rx = gamepad1.right_stick_x;
//
//        double theta = Math.atan2(y, x);
//        double power = Math.hypot(x, y);
//
//        double sin = Math.sin(theta - Math.PI/4);
//        double cos = Math.cos(theta - Math.PI/4);
//        double max = Math.max(Math.abs(sin), Math.abs(cos));
//
//        double frontLeftPower = power * cos/max + rx;
//        double frontRightPower = power * sin/max - rx;
//        double backLeftPower = power * sin/max + rx;
//        double backRightPower = power * cos/max - rx;
//
//        if ((power + Math.abs(rx)) > 1) {
//            frontLeftPower /= power + Math.abs(rx);
//            frontRightPower /= power + Math.abs(rx);
//            backLeftPower /= power + Math.abs(rx);
//            backRightPower /= power + Math.abs(rx);
//        }
//
//        frontLeftMotor.setPower(frontLeftPower);
//        frontRightMotor.setPower(frontRightPower);
//        backLeftMotor.setPower(backLeftPower);
//        backRightMotor.setPower(backRightPower);
//    }
//
//    private void handleModeSelection() {
//        if (gamepad1.b) {
//            currentColorMode = ColorMode.GREEN_PURPLE_PURPLE;
//            telemetry.addData("顏色模式", "GREEN_PURPLE_PURPLE");
//            sleep(200);
//        } else if (gamepad1.x) {
//            currentColorMode = ColorMode.PURPLE_GREEN_PURPLE;
//            telemetry.addData("顏色模式", "PURPLE_GREEN_PURPLE");
//            sleep(200);
//        } else if (gamepad1.y) {
//            currentColorMode = ColorMode.PURPLE_PURPLE_GREEN;
//            telemetry.addData("顏色模式", "PURPLE_PURPLE_GREEN");
//            sleep(200);
//        }
//    }
//
//    private void handleShooterControls() {
//        // 右扳機控制發射馬達功率
//        shooterMotor.setPower(gamepad1.right_trigger);
//
//        // 肩鍵控制發射角度
//        if (gamepad1.left_bumper) {
//            angleServo.setPower(0.1);
//        } else if (gamepad1.right_bumper) {
//            angleServo.setPower(-0.1);
//        } else {
//            angleServo.setPower(0);
//        }
//    }
//
//    private void updateBasePosition() {
//        if (intakeMode) {
//            rotationBaseServo.setPosition(INTAKE_POSITIONS[currentBasePosition]);
//        } else {
//            rotationBaseServo.setPosition(BASE_POSITIONS[currentBasePosition]);
//        }
//        sleep(BASE_ROTATION_DELAY_MS);
//    }
//
//    private void rotateToPosition(int position) {
//        if (position < 0 || position > 2) return;
//
//        currentBasePosition = position;
//        if (intakeMode) {
//            rotationBaseServo.setPosition(INTAKE_POSITIONS[position]);
//        } else {
//            rotationBaseServo.setPosition(BASE_POSITIONS[position]);
//        }
//
//        // 等待底盤轉動完成
//        telemetry.addData("底盤轉動", "轉動到位置 " + (position + 1));
//        telemetry.update();
//        sleep(BASE_ROTATION_DELAY_MS);
//    }
//
//    private void fireAtCurrentPosition() {
//        telemetry.addData("發射", "在位置 " + (currentBasePosition + 1) + " 發射");
//        telemetry.update();
//
//        // 啟動發射servo
//        firingServo.setPosition(FIRING_SERVO_ACTIVE);
//        sleep(FIRING_DELAY_MS);
//
//        // 返回原位
//        firingServo.setPosition(FIRING_SERVO_REST);
//        sleep(200); // 短暫等待確保返回
//    }
//
//    private DetectedColor detectColorAtPosition(int position) {
//        // 先轉動到底盤位置
//        rotateToPosition(position);
//
//        // 等待感測器穩定
//        sleep(SENSOR_READ_DELAY_MS);
//
//        // 讀取顏色
//        return getPositionColor(position);
//    }
//
//    private DetectedColor getPositionColor(int position) {
//        switch (position) {
//            case 0:
//                return getCombinedColor(colorSensor0, colorSensor1, "位置1");
//            case 1:
//                return getCombinedColor(colorSensor2, colorSensor3, "位置2");
//            case 2:
//                return getCombinedColor(colorSensor4, colorSensor5, "位置3");
//            default:
//                return DetectedColor.UNKNOWN;
//        }
//    }
//
//    private DetectedColor getCombinedColor(NormalizedColorSensor sensor1, NormalizedColorSensor sensor2,
//                                           String positionName) {
//        DetectedColor color1 = getSingleSensorColor(sensor1);
//        DetectedColor color2 = getSingleSensorColor(sensor2);
//
//        // 優先返回確定的顏色
//        if (color1 == DetectedColor.PURPLE || color2 == DetectedColor.PURPLE) {
//            return DetectedColor.PURPLE;
//        }
//        if (color1 == DetectedColor.GREEN || color2 == DetectedColor.GREEN) {
//            return DetectedColor.GREEN;
//        }
//
//        return DetectedColor.UNKNOWN;
//    }
//
//    private DetectedColor getSingleSensorColor(NormalizedColorSensor sensor) {
//        NormalizedRGBA color = sensor.getNormalizedColors();
//        float red = color.red;
//        float green = color.green;
//        float blue = color.blue;
//
//        float total = red + green + blue;
//        if (total < 0.01f) return DetectedColor.UNKNOWN;
//
//        float redRatio = red / total;
//        float greenRatio = green / total;
//        float blueRatio = blue / total;
//
//        // 檢測紫色
//        boolean isPurple = (blueRatio > 0.35f) && (redRatio > 0.25f) &&
//                (redRatio < 0.4f) && ((blue - green) > 0.05f);
//
//        // 檢測綠色
//        boolean isGreen = (greenRatio > 0.4f) && (blueRatio < 0.35f) &&
//                (redRatio < 0.35f) && ((green - blue) > 0.02f);
//
//        if (isPurple) return DetectedColor.PURPLE;
//        if (isGreen) return DetectedColor.GREEN;
//        return DetectedColor.UNKNOWN;
//    }
//
//    private void executeAutoFiringSequence() {
//        // 切換到射擊模式
//        intakeMode = false;
//        updateBasePosition();
//
//        telemetry.addLine("=== 開始自動發射序列 ===");
//        telemetry.addData("顏色模式", currentColorMode);
//        telemetry.update();
//
//        // 檢測三個位置的顏色
//        DetectedColor[] colors = new DetectedColor[3];
//        for (int i = 0; i < 3; i++) {
//            colors[i] = detectColorAtPosition(i);
//            telemetry.addData("位置 " + (i + 1), colors[i]);
//            telemetry.update();
//        }
//
//        // 決定發射順序
//        int[] firingOrder = determineFiringOrder(colors);
//
//        // 執行發射
//        telemetry.addLine("--- 執行發射順序 ---");
//        telemetry.update();
//
//        for (int i = 0; i < firingOrder.length; i++) {
//            if (firingOrder[i] != -1) {
//                rotateToPosition(firingOrder[i]);
//                fireAtCurrentPosition();
//
//                if (i < firingOrder.length - 1) {
//                    sleep(FIRING_INTERVAL_MS);
//                }
//            }
//        }
//
//        telemetry.addLine("=== 發射序列完成 ===");
//        telemetry.update();
//
//        // 返回第一個位置
//        rotateToPosition(0);
//    }
//
//    private int[] determineFiringOrder(DetectedColor[] colors) {
//        int[] firingOrder = {-1, -1, -1};
//        boolean[] positionUsed = new boolean[3];
//        int orderIndex = 0;
//
//        // 獲取理想顏色順序
//        DetectedColor[] idealOrder = getIdealColorOrder();
//
//        // 第一步：按照理想順序發射有球的顏色
//        for (DetectedColor targetColor : idealOrder) {
//            for (int i = 0; i < colors.length; i++) {
//                if (!positionUsed[i] && colors[i] == targetColor) {
//                    firingOrder[orderIndex++] = i;
//                    positionUsed[i] = true;
//                    break;
//                }
//            }
//        }
//
//        // 第二步：發射剩餘位置
//        for (int i = 0; i < colors.length; i++) {
//            if (!positionUsed[i]) {
//                firingOrder[orderIndex++] = i;
//                positionUsed[i] = true;
//            }
//        }
//
//        return firingOrder;
//    }
//
//    private DetectedColor[] getIdealColorOrder() {
//        switch (currentColorMode) {
//            case GREEN_PURPLE_PURPLE:
//                return new DetectedColor[]{DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE};
//            case PURPLE_GREEN_PURPLE:
//                return new DetectedColor[]{DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE};
//            case PURPLE_PURPLE_GREEN:
//                return new DetectedColor[]{DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN};
//            default:
//                return new DetectedColor[]{DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN};
//        }
//    }
//
//    private void updateTelemetry() {
//        telemetry.addLine("=== 機器人狀態 ===");
//        telemetry.addData("當前模式", currentColorMode);
//        telemetry.addData("操作模式", intakeMode ? "進料模式" : "射擊模式");
//        telemetry.addData("底盤位置", "位置 " + (currentBasePosition + 1));
//        telemetry.addData("序列狀態", sequenceInProgress ? "進行中" : "準備就緒");
//
//        if (!sequenceInProgress) {
//            // 顯示當前位置的顏色
//            DetectedColor currentColor = getPositionColor(currentBasePosition);
//            telemetry.addData("當前顏色", currentColor);
//
//            // 顯示發射系統狀態
//            telemetry.addData("發射馬達功率", "%.2f", shooterMotor.getPower());
//            telemetry.addData("發射角度控制", angleServo.getPower() > 0 ? "上調" :
//                    (angleServo.getPower() < 0 ? "下調" : "靜止"));
//        }
//
//        telemetry.addData("控制提示", "A - 自動發射, B/X/Y - 顏色模式");
//        telemetry.addData("控制提示", "D-pad Up - 切換進料/射擊");
//        telemetry.addData("控制提示", "右扳機 - 發射功率, 肩鍵 - 調整角度");
//        telemetry.update();
//    }
//}