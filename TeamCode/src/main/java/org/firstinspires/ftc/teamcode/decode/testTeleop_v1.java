package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testTeleop_v1")
public class testTeleop_v1 extends LinearOpMode {

    NormalizedColorSensor colorSensor;

    // 根據您的新數據調整的極敏感閾值
    private static final float MIN_COLOR_THRESHOLD = 0.0005f; // 非常低的閾值

    // Servo 位置
    private static final double SERVO_UP_POSITION = 0.8;
    private static final double SERVO_DOWN_POSITION = 0.2;

    private boolean servoActivated = false;
    private int detectionCount = 0;
    private static final int REQUIRED_DETECTIONS = 3; // 需要連續檢測到多次才觸發

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");

    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();

        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        telemetry.addData("Red", "%.4f", red); // 顯示4位小數
        telemetry.addData("Green", "%.4f", green);
        telemetry.addData("Blue", "%.4f", blue);

        // 檢查數據是否有效
        if (red < MIN_COLOR_THRESHOLD && green < MIN_COLOR_THRESHOLD && blue < MIN_COLOR_THRESHOLD) {
            telemetry.addData("Status", "Signal too weak");
            return DetectedColor.UNKNOWN;
        }

        // 修正：交換綠色和紫色的判斷邏輯
        // 紫色物體的藍色值最高
        if (blue > 0.0035f && blue > green && blue > red) {
            telemetry.addData("Logic", "PURPLE detected - Blue is highest");
            return DetectedColor.PURPLE;
        }
        // 綠色物體的綠色值最高
        else if (green > 0.0025f && green > blue && green > red) {
            telemetry.addData("Logic", "GREEN detected - Green is highest");
            return DetectedColor.GREEN;
        }

        return DetectedColor.UNKNOWN;
    }

    // 更精確的比率方法 - 修正版本
    public DetectedColor getDetectedColorByRatio(Telemetry telemetry) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();

        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        telemetry.addData("Red", "%.4f", red);
        telemetry.addData("Green", "%.4f", green);
        telemetry.addData("Blue", "%.4f", blue);

        // 計算總和
        float sum = red + green + blue;
        if (sum < 0.002f) { // 總和太小
            telemetry.addData("Status", "Signal too weak - sum: %.4f", sum);
            return DetectedColor.UNKNOWN;
        }

        // 計算比率
        float redRatio = red / sum;
        float greenRatio = green / sum;
        float blueRatio = blue / sum;

        telemetry.addData("Red %", "%.1f%%", redRatio * 100);
        telemetry.addData("Green %", "%.1f%%", greenRatio * 100);
        telemetry.addData("Blue %", "%.1f%%", blueRatio * 100);

        // 修正：交換綠色和紫色的比率判斷
        // 紫色物體：藍色比例 > 45%
        if (blueRatio > 0.45f && blueRatio > greenRatio && blueRatio > redRatio) {
            telemetry.addData("Logic", "PURPLE - Blue ratio: %.1f%%", blueRatio * 100);
            return DetectedColor.PURPLE;
        }
        // 綠色物體：綠色比例 > 40%
        else if (greenRatio > 0.40f && greenRatio > blueRatio && greenRatio > redRatio) {
            telemetry.addData("Logic", "GREEN - Green ratio: %.1f%%", greenRatio * 100);
            return DetectedColor.GREEN;
        }

        return DetectedColor.UNKNOWN;
    }

    // 增強檢測方法 - 使用多種條件（修正版本）
    public DetectedColor getDetectedColorEnhanced(Telemetry telemetry) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();

        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        telemetry.addData("Raw - R", "%.4f", red);
        telemetry.addData("Raw - G", "%.4f", green);
        telemetry.addData("Raw - B", "%.4f", blue);

        // 修正：交換條件檢測
        boolean possiblePurple = (blue > 0.003f) && (blue > green) && (blue > red);
        boolean possibleGreen = (green > 0.002f) && (green > blue) && (green > red);

        // 檢查信號強度
        float maxVal = Math.max(Math.max(red, green), blue);
        boolean hasGoodSignal = maxVal > 0.002f;

        telemetry.addData("Max Value", "%.4f", maxVal);
        telemetry.addData("Good Signal", hasGoodSignal);
        telemetry.addData("Possible Purple", possiblePurple);
        telemetry.addData("Possible Green", possibleGreen);

        if (hasGoodSignal) {
            if (possiblePurple && !possibleGreen) {
                return DetectedColor.PURPLE;
            } else if (possibleGreen && !possiblePurple) {
                return DetectedColor.GREEN;
            }
        }

        return DetectedColor.UNKNOWN;
    }

    // 簡單檢測方法 - 修正版本
    public DetectedColor getDetectedColorSimple(Telemetry telemetry) {
        NormalizedRGBA color = colorSensor.getNormalizedColors();

        float red = color.red;
        float green = color.green;
        float blue = color.blue;

        telemetry.addData("R", "%.4f", red);
        telemetry.addData("G", "%.4f", green);
        telemetry.addData("B", "%.4f", blue);

        // 最簡單的邏輯：哪個顏色值最高就判斷為什麼顏色（修正版本）
        float maxValue = Math.max(Math.max(red, green), blue);

        // 修正：交換判斷條件
        if (maxValue == blue && blue > 0.003f) {
            telemetry.addData("Logic", "PURPLE - Blue is highest");
            return DetectedColor.PURPLE;
        }
        else if (maxValue == green && green > 0.002f) {
            telemetry.addData("Logic", "GREEN - Green is highest");
            return DetectedColor.GREEN;
        }

        return DetectedColor.UNKNOWN;
    }





    @Override
    public void runOpMode() {
        init(hardwareMap);

        telemetry.addData("Status", "Initialized - Color Logic Corrected");
        telemetry.addData("Detection Logic", "High Blue = PURPLE, High Green = GREEN");
        telemetry.addData("Controls", "A - Reset, X - Test Servo, Y - Switch Method");
        telemetry.update();
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
        CRServo xuanzhuan = hardwareMap.get(CRServo.class,"servo0");
        Servo leftPushServo = hardwareMap.get(Servo.class,"servo1");
        Servo rightPushServo = hardwareMap.get(Servo.class,"servo2");

        int detectionMethod = 0; // 0: Enhanced, 1: Simple, 2: Ratio


        boolean purple = false;
        boolean green = false;
        double leftPushON = 1;
        double leftPushOFF = 0;
        double rightPushON = 1;
        double rightPushOFF = 0;
        long startTime1 = 0;
        long currentTime1 = 0;
        long startTime2 = 0;
        long currentTime2 = 0;
        long startTime3 = 0;
        long currentTime3 = 0;
        long startTime4 = 0;
        long currentTime4 = 0;
        long startTime5 = 0;
        long currentTime5 = 0;
        long startTime6 = 0;
        long currentTime6 = 0;
        long startTime7 = 0;
        long currentTime7 = 0;
        long startTime = System.currentTimeMillis();

        intakeMotor.setPower(0);
        leftPushServo.setPosition(leftPushOFF);
        rightPushServo.setPosition(rightPushOFF);
        xuanzhuan.setPower(0);
        purple = false;
        green = false;



        waitForStart();

        while (opModeIsActive()) {
            DetectedColor detectedColor;

            // 根據選擇的方法進行檢測
            switch (detectionMethod) {
                case 0:
                    detectedColor = getDetectedColorEnhanced(telemetry);
                    telemetry.addData("Method", "Enhanced");
                    break;
                case 1:
                    detectedColor = getDetectedColorSimple(telemetry);
                    telemetry.addData("Method", "Simple");
                    break;
                case 2:
                    detectedColor = getDetectedColorByRatio(telemetry);
                    telemetry.addData("Method", "Ratio");
                    break;
                default:
                    detectedColor = getDetectedColorEnhanced(telemetry);
                    telemetry.addData("Method", "Enhanced (default)");
                    break;
            }

            telemetry.addData("Detected Color", detectedColor);
            if (detectedColor == DetectedColor.PURPLE) {
                purple = true;
                startTime3 = System.currentTimeMillis();
            } else if(detectedColor == DetectedColor.GREEN) {
                green = true;
                startTime4 = System.currentTimeMillis();
            }



            if(purple){
                currentTime3 = System.currentTimeMillis() - startTime3;
//                if(currentTime3>5000){
//                    purple = false;
//                }
            }

            if(green){
                currentTime4 = System.currentTimeMillis() - startTime4;
//                if(currentTime4>5000){
//                    green = false;
//                }
            }
            if(!gamepad1.x && !gamepad1.y){
                if(gamepad1.left_bumper){
                    if(purple){
                        leftPushServo.setPosition(leftPushON);
                    }else if(green){
                        rightPushServo.setPosition(rightPushON);
                    }
                    purple = false;
                    green = false;

                }

                if(!purple && !green){
                    leftPushServo.setPosition(leftPushOFF);
                    rightPushServo.setPosition(rightPushOFF);
                }
            }

            if(gamepad1.x){
                leftPushServo.setPosition(leftPushON);
            }if(gamepad1.y){
                rightPushServo.setPosition(rightPushON);
            }






            if(gamepad1.a){
                intakeMotor.setPower(1);
                xuanzhuan.setPower(0.8);
            }else if(gamepad1.b){
                intakeMotor.setPower(0);
                xuanzhuan.setPower(0);
            }
//
//
//

            telemetry.addData("purple",purple);
            telemetry.addData("green",green);
            telemetry.addData("xuanzhuan",xuanzhuan.getPower());
            telemetry.addData("leftPushServo",leftPushServo.getPosition());
            telemetry.addData("rightPushServo",rightPushServo.getPosition());
            telemetry.addData("intakeMotor",intakeMotor.getPower());



            telemetry.update();


        }
    }
}


