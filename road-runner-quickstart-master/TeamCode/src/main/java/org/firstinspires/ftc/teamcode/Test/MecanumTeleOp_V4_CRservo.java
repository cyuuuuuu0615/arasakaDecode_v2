package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class MecanumTeleOp_V4_CRservo extends LinearOpMode {
    ColorSensor colorSensor;
    public static boolean inRange(double number, double start, double end) {
        return number >= start && number <= end;
    }


    public static double handleRange(double number, double start, double end) {
        if (number < start) {
            return start;
        } else if (number > end) {
            return end;
        } else {
            return number;
        }
    }





    @Override
    public void runOpMode() throws InterruptedException {
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        colorSensor = hardwareMap.get(ColorSensor.class, "I2C1");
//        ColorSensor show = hardwareMap.get(ColorSensor.class,"show");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);
//        show.enableLed(false);







        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor1");
        DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor2");
        DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor0");
        DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor3");
        DcMotorEx lefthand = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
        DcMotorEx xuangua = (DcMotorEx) hardwareMap.dcMotor.get("motor5");
        Servo zuoshoubi = hardwareMap.get(Servo.class,"servo6");
        Servo youshoubi = hardwareMap.get(Servo.class,"servo2");
//        Servo jiazixiaobi = hardwareMap.get(Servo.class,"servo2");
//        Servo shoubi = hardwareMap.get(Servo.class,"servo6");
//        Servo shouzhang = hardwareMap.get(Servo.class,"servo3");
        Servo chanzi = hardwareMap.get(Servo.class,"servo7");
        Servo shouzhang0 = hardwareMap.get(Servo.class,"servo0");
        Servo shouzhang1 = hardwareMap.get(Servo.class,"servo1");
        CRServo zhuazi0 = hardwareMap.get(CRServo.class,"servo8");
        CRServo zhuazi1 = hardwareMap.get(CRServo.class,"servo9");

//        zhuazi0.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lefthand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lefthand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xuangua.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xuangua.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xuangua.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        xuangua.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthand.setDirection(DcMotorSimple.Direction.REVERSE);

        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        if (isStopRequested()) return;
        if (opModeIsActive()){
            double zuoshouhandup = 0.5222;
            double zuoshouhanddon = 1;
            double zuoshouhandping = 0.9;
            double youshouhandup = 1 - zuoshouhandup;
            double youshouhanddon = 1 - zuoshouhanddon;
            double youshouhandping = 1 - zuoshouhandping;
            double shuzhangop = 1;
            double shouzhanged = 0;
            double handup = 1;
            double handdon = 0;
            double handping = 0.9;
            double shuzhang0op = 0;
            double shouzhang0ed = 1;
            double shuzhang1op = 1;
            double shouzhang1ed = 0;
            double chanziping = 0.4778;
            double chanzidown = 0.45;
            double chanziup = 1;
            double handchanziup = 0.6104;
            double jiazixiaobichanziup = 0;
            double jiazixiaobidown = 1;
            boolean isSequenceStarted = false;
            boolean isSequenceStarted1 = false;
            boolean isSequenceStarted5 = false;
            long startTime = 0;
            long startTime1 = 0;
            long startTime5 = 0;
            int lefthandlastposition = 5;
            int one = 100;
            int two = 233;
            int lastxuanguaposition = 0;
            boolean mode = false;
            boolean jiazi = false;
            boolean get = false;
//            boolean updown = false;
//            shoubi.setPosition(handping);
////            shouzhang.setPosition(shuzhangop);
//            jiazixiaobi.setPosition(jiazixiaobidown);
            chanzi.setPosition(chanziping);
//            shouzhang0.setPosition(shouzhang0ed);
//            shouzhang0.setPosition(shouzhang0ed);
            while (opModeIsActive()) {
                Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

                if (bCurrState && (bCurrState != bPrevState))  {

                    // button is transitioning to a pressed state. So Toggle LED
                    bLedOn = !bLedOn;
                    colorSensor.enableLed(bLedOn);
                }
                bPrevState = bCurrState;
                // send the info back to driver station using telemetry function.
//                telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Clear", colorSensor.alpha());
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());
                telemetry.addData("Hue", hsvValues[0]);

//                show.enableLed(colorSensor.alpha() > 2000);


                int xuanguacp= xuangua.getCurrentPosition();


                if(gamepad2.cross){
//                    shoubi.setPosition(handping);
                    zuoshoubi.setPosition(zuoshouhandping);
                    youshoubi.setPosition(youshouhandping);
                    jiazi = false;
                    get = false;
//                    shouzhang.setPosition(shuzhangop);
                }




                if(gamepad2.triangle){
//                    shoubi.setPosition(handdon);
                    jiazi = true;
                    get = false;
                }
                if(jiazi && !isSequenceStarted && !isSequenceStarted1){
                    if(!inRange(colorSensor.red(),600,1300)){
                        zhuazi1.setPower(1);
                        zhuazi0.setPower(-1);
                    }
                    else {
                        jiazi = false;
                        get = true;
                    }
                }
                else if(!jiazi && !isSequenceStarted && !isSequenceStarted1) {
                    zhuazi1.setPower(0);
                    zhuazi0.setPower(0);
                    if(get){
//                        jiazixiaobi.setPosition(jiazixiaobichanziup);
//                        shoubi.setPosition(handping);
                    }
                }
                if (gamepad2.square) {
                    isSequenceStarted = true;
                    startTime = System.currentTimeMillis();
                }
                if (isSequenceStarted) {
                    long currentTime = System.currentTimeMillis() - startTime;

                    if (currentTime < 1000) {
                        zhuazi1.setPower(-1);
                        zhuazi0.setPower(1);
                    }
                    else {
                        isSequenceStarted = false;
                        get = false;
                        jiazi = false;
                    }
                }

                if(gamepad2.circle){
                    isSequenceStarted1 = true;
                    startTime1 = System.currentTimeMillis();
                }
                if(isSequenceStarted1){
                    long currentTime1 = System.currentTimeMillis() - startTime1;
                    if(currentTime1 < 500){
                        zuoshoubi.setPosition(zuoshouhandup);
                        youshoubi.setPosition(youshouhandup);
                    }
                    else if(currentTime1 < 1500) {
                        zhuazi1.setPower(-1);
                        zhuazi0.setPower(1);
                    } else if (currentTime1 < 2000) {
                        zuoshoubi.setPosition(zuoshouhandping);
                        youshoubi.setPosition(youshouhandping);
                    }
                    else{
                        isSequenceStarted1 = false;
                        get = false;
                    }
                }
                if(gamepad1.right_bumper){
                    shouzhang0.setPosition(shuzhang0op);
                    shouzhang1.setPosition(shuzhang1op);
                }
                else if(gamepad1.left_bumper){
                    shouzhang0.setPosition(shouzhang0ed);
                    shouzhang1.setPosition(shouzhang1ed);
                }
                if (gamepad1.x){
                    isSequenceStarted5 = true;
                    startTime5 = System.currentTimeMillis();
                }
                if (isSequenceStarted5){
                    long currentTime5 = System.currentTimeMillis() - startTime5;
                    if (currentTime5 < 1000){
                        chanzi.setPosition(chanziup);
                    }
                    else if(currentTime5 < 2000){
                        chanzi.setPosition(chanziping);
                    }
                    else{
                        isSequenceStarted5 = false;
                    }
                }
                if((gamepad1.left_trigger - gamepad1.right_trigger) != 0){
                    xuangua.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lastxuanguaposition = xuanguacp;
                    if(xuangua.getCurrentPosition() >= 3000){
                        xuangua.setPower(handleRange((gamepad1.left_trigger - gamepad1.right_trigger),-1,0));
                        telemetry.addLine("downonly");
                    }
                    else if(xuangua.getCurrentPosition() <= 5){
                        xuangua.setPower(handleRange((gamepad1.left_trigger - gamepad1.right_trigger),0,1));
                        telemetry.addLine("uponly");
                    }
                    else {
                        xuangua.setPower((gamepad1.left_trigger - gamepad1.right_trigger));
                        telemetry.addLine("up and down");

                    }
                }else {
                    xuangua.setTargetPosition(lastxuanguaposition);
                    xuangua.setPower(1);
                    xuangua.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if(lefthand.getCurrentPosition() > 1600){
                    lefthand.setPower(handleRange((gamepad2.left_trigger - gamepad2.right_trigger),-1,0));

                }
                else if(lefthand.getCurrentPosition() <= 10){
                    lefthand.setPower(handleRange((gamepad2.left_trigger - gamepad2.right_trigger),0,1));
                }
                else {
                    lefthand.setPower(handleRange(gamepad2.left_trigger - gamepad2.right_trigger,-1, 1));

                }
//                if((gamepad2.left_trigger - gamepad2.right_trigger) != 0){
//                    lefthand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    lefthandlastposition = lefthand.getCurrentPosition();
//                }else {
//                    lefthand.setPower(1);
//                    lefthand.setTargetPosition(lefthandlastposition);
//                    lefthand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
                double x = gamepad2.left_stick_x;
                double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
                //double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad2.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y + x - rx) / denominator;
//            double backRightPower = (y - x - rx) / denominator;
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

                telemetry.addData("Left stick X", x);
                telemetry.addData("Front Left Power", frontLeftPower);
                telemetry.addData("Back Left Power", backLeftPower);
                telemetry.addData("lefthand",lefthand.getCurrentPosition());
                telemetry.addData("gamepad", (gamepad2.left_trigger - gamepad2.right_trigger));

                telemetry.addData("Front Right Power", frontRightPower);
                telemetry.addData("Back Right Power", backRightPower);

                telemetry.addData("xuangua power",xuangua.getPower());
                telemetry.addData("xuangua position",xuanguacp);

                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });

                telemetry.update();
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }


    }
}
