package org.firstinspires.ftc.teamcode.TeleOp;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.app.Activity;
import android.graphics.Color;
import android.view.View;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
 *
 * This OpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The OpMode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "MecanumBlue_nolimit", group = "TeleOOP")

public class b2 extends LinearOpMode {

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


    ColorSensor colorSensor;// Hardware Device Object
    boolean limit = true;
    double lefthandin = 0.3;
    double lefthandout = 1;
    double righthandin = 1 - lefthandin;
    double righthandout = 1 - lefthandout;
    double shuzhangop = 1;
    double shouzhanged = 0;
    double zuoshouhandup = 0;
    double zuoshouhanddon = 0.51;
    double zuoshouhandping = 0.4;
    double youshouhandup = 1 - zuoshouhandup;
    double youshouhanddon = 1 - zuoshouhanddon;
    double youshouhandping = 1 - zuoshouhandping;
    double shuzhang0op = 0;
    double shouzhang0ed = 1;
    double shuzhang1op = 1;
    double shouzhang1ed = 0;
    double chanziping = 0;
    double chanzidown = 0.45;
    double chanziup = 0.53;
    double handchanziup = 0.6104;
    double jiazixiaobichanziup = 0;
    double jiazixiaobidown = 1;
    boolean isSequenceStarted = false;
    boolean outSequenStarted = false;
    boolean inSequenStarted = false;
    boolean isSequenceStarted1 = false;
    boolean isSequenceStarted5 = false;
    boolean inout = true;
    long startTime = 0;
    long startTimeout = 0;
    long startTimemin = 0;
    long startTime1 = 0;
    long startTime5 = 0;
    int lefthandlastposition = 5;
    int lower_limit = 0;
    int one = 100;
    int two = 233;
    int lastxuanguaposition = 0;
    boolean mode = false;
    boolean jiazizheng = false;
    boolean jiazifan = false;
    boolean get = false;
    boolean yellow = true;
    boolean setting = true;
    boolean shenshou  = false;
    boolean updown = true;
    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
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
//__________________________________________________________________________
        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "I2C1");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        // wait for the start button to be pressed.
        DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor1");
        DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor2");
        DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor0");
        DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor3");
//        DcMotorEx lefthand = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
        DcMotorEx xuangua = (DcMotorEx) hardwareMap.dcMotor.get("motor5");
//    DcMotorEx shoubi = (DcMotorEx) hardwareMap.dcMotor.get("motor6");
        Servo jiazixiaobi = hardwareMap.get(Servo.class,"servo2");
        Servo zuoshoubi = hardwareMap.get(Servo.class,"servo6");
        Servo youshoubi = hardwareMap.get(Servo.class,"servo11");
        Servo lefthand = hardwareMap.get(Servo.class,"servo0");
        Servo righthand = hardwareMap.get(Servo.class,"servo5");
//        Servo shouzhang = hardwareMap.get(Servo.class,"servo3");
        Servo chanzi = hardwareMap.get(Servo.class,"servo7");
        Servo saodijiqiren  = hardwareMap.get(Servo.class,"servo3");
        Servo shouzhang0 = hardwareMap.get(Servo.class,"servo4");
        Servo shouzhang1 = hardwareMap.get(Servo.class,"servo1");
        CRServo zhuazi0 = hardwareMap.get(CRServo.class,"servo8");
        CRServo zhuazi1 = hardwareMap.get(CRServo.class,"servo9");

//        zhuazi0.setDirection(DcMotorSimple.Direction.REVERSE);
//        lefthand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lefthand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lefthand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xuangua.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xuangua.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xuangua.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //xuangua.setDirection(DcMotorSimple.Direction.REVERSE);
//        lefthand.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // while the OpMode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        if(opModeIsActive()) {
            chanzi.setPosition(chanziping);
            zuoshoubi.setPosition(zuoshouhandping);
            youshoubi.setPosition(youshouhandping);
            while (opModeIsActive()) {
                if(!isSequenceStarted1){
                    if(!updown){
                        if (shenshou){
                            zuoshoubi.setPosition(zuoshouhanddon);
                            youshoubi.setPosition(youshouhanddon);
                        }else {
                            zuoshoubi.setPosition(zuoshouhanddon + 0.01);
                            youshoubi.setPosition(youshouhanddon - 0.01);
                        }
                    }else{
                        zuoshoubi.setPosition(zuoshouhandping);
                        youshoubi.setPosition(youshouhandping);
                    }
                }
                saodijiqiren.setPosition(handleRange(gamepad1.left_stick_x,0,0.5));
                if(gamepad2.ps){
                    yellow = !yellow;
                }
                if(yellow){
                    telemetry.addLine("YELLOW AND BLUE");
                }else {
                    telemetry.addLine("BLUE ONLY");
                }
                int xuanguacp = xuangua.getCurrentPosition();
                // check the status of the x button on either gamepad.
                //      bCurrState = gamepad1.x;

                // check for button state transitions.
                if (bCurrState && (bCurrState != bPrevState)) {

                    // button is transitioning to a pressed state. So Toggle LED
                    bLedOn = !bLedOn;
                    colorSensor.enableLed(bLedOn);
                }

                // update previous state variable.
                bPrevState = bCurrState;

                // convert the RGB values to HSV values.
                Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

                // send the info back to driver station using telemetry function.
                telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Clear", colorSensor.alpha());
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());
                telemetry.addData("Hue", hsvValues[0]);

                if(gamepad2.cross){
                    updown = true;
//                    shoubi.setPosition(handping);
                    zuoshoubi.setPosition(zuoshouhandping);
                    youshoubi.setPosition(youshouhandping);
                    jiazizheng = false;
                    jiazifan = false;
                    get = false;
//                    shouzhang.setPosition(shuzhangop);
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
                        zuoshoubi.setPosition(zuoshouhandping);
                        youshoubi.setPosition(youshouhandping);
                        isSequenceStarted = false;
                        get = false;
                        jiazizheng = false;
                        jiazifan = false;
                    }
                }


                if (gamepad2.left_bumper) {
                    updown = false;
                    zuoshoubi.setPosition(zuoshouhanddon);
                    youshoubi.setPosition(youshouhanddon);
                    jiazizheng = true;
                    jiazifan = false;
                    get = false;


                } else if (gamepad2.right_bumper) {
                    updown = false;
//                    zuoshoubi.setPosition(zuoshouhanddon);
//                    youshoubi.setPosition(youshouhanddon);
                    jiazizheng = false;
                    jiazifan = true;
                    get = false;

                }
                if (jiazizheng && !isSequenceStarted && !isSequenceStarted1 && !jiazifan) {
                    if(!yellow) {
                        if (colorSensor.blue() < 300) {
                            zhuazi1.setPower(1);
                            zhuazi0.setPower(-1);
                        } else {
                            jiazizheng = false;
                            get = true;
                        }
                    }else {
                        if ((colorSensor.red() > 400) || (colorSensor.blue() > 300)) {
                            jiazizheng = false;
                            get = true;
                            zhuazi1.setPower(0);
                            zhuazi0.setPower(0);
                        } else {
                            zhuazi1.setPower(1);
                            zhuazi0.setPower(-1);
                        }
                    }
                } else if (!jiazizheng && !isSequenceStarted && !isSequenceStarted1 && !jiazifan) {
                    zhuazi1.setPower(0);
                    zhuazi0.setPower(0);
                    if (get) {

//                        zuoshoubi.setPosition(zuoshouhandping);
//                        youshoubi.setPosition(youshouhandping);
                        telemetry.addLine("get");
                    }
                }
                if (jiazifan && !isSequenceStarted && !isSequenceStarted1 && !jiazizheng) {
                    if(!yellow) {
                        if (colorSensor.blue() < 300) {
                            zhuazi1.setPower(-1);
                            zhuazi0.setPower(1);
                        } else {
                            jiazizheng = false;
                            get = true;
                        }
                    }else {
                        if ((colorSensor.red() > 400) || (colorSensor.blue() > 300)) {
                            jiazizheng = false;
                            get = true;
                            zhuazi1.setPower(0);
                            zhuazi0.setPower(0);
                        } else {
                            zhuazi1.setPower(-1);
                            zhuazi0.setPower(1);
                        }
                    }
                } else if (!jiazifan  && !isSequenceStarted && !isSequenceStarted1 && !jiazizheng) {
                    zhuazi1.setPower(0);
                    zhuazi0.setPower(0);
                    if (get) {
                        jiazixiaobi.setPosition(jiazixiaobichanziup);
//                        zuoshoubi.setPosition(zuoshouhanddon);
//                        youshoubi.setPosition(youshouhandping);
                        telemetry.addLine("get");
                    }
                }

                if(gamepad2.circle){
                    updown = true;
                    shenshou = false;
                    isSequenceStarted1 = true;
                    startTime1 = System.currentTimeMillis();
                    jiazizheng = false;
                    jiazifan = false;
                }
                //back
                if(isSequenceStarted1){
                    long currentTime1 = System.currentTimeMillis() - startTime1;
                    lefthand.setPosition(0);
                    righthand.setPosition(1);
//                    lefthand.setTargetPosition(20);
//                    lefthand.setPower(1);
//                    lefthand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                if(gamepad1.right_bumper){
                    shouzhang0.setPosition(shuzhang0op);
                    shouzhang1.setPosition(shuzhang1op);
                }
                else if(gamepad1.left_bumper){
                    shouzhang0.setPosition(shouzhang0ed);
                    shouzhang1.setPosition(shouzhang1ed);
                }
                if(gamepad1.y){
                    limit = false;
                } else if (gamepad1.x) {
                    limit = true;
                }
                if(gamepad1.b){
                    lower_limit = xuangua.getCurrentPosition();
                    limit = true;

                }
                if((gamepad1.left_trigger - gamepad1.right_trigger) != 0){
                    xuangua.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lastxuanguaposition = xuanguacp;
                    if(limit) {
                        if (xuangua.getCurrentPosition() >= lower_limit + 2150) {
                            xuangua.setPower(handleRange((gamepad1.left_trigger - gamepad1.right_trigger), -1, 0));
                            telemetry.addLine("downonly");
                        } else if (xuangua.getCurrentPosition() <= lower_limit) {
                            xuangua.setPower(handleRange((gamepad1.left_trigger - gamepad1.right_trigger), 0, 1));
                            telemetry.addLine("uponly");
                        } else {
                            xuangua.setPower((gamepad1.left_trigger - gamepad1.right_trigger));
                            telemetry.addLine("up and down");

                        }
                    }else{
                        xuangua.setPower((gamepad1.left_trigger - gamepad1.right_trigger));
                        telemetry.addLine("no_limit");
                    }
                }else {
                    xuangua.setTargetPosition(lastxuanguaposition);
                    xuangua.setPower(1);
                    xuangua.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if(!isSequenceStarted1){
                    if(inout) {
                        if (gamepad2.left_trigger > 0) {
                            startTimeout = System.currentTimeMillis();
                            outSequenStarted = true;
                        } else if (gamepad2.right_trigger > 0) {
                            startTimemin = System.currentTimeMillis();
                            inSequenStarted = true;
                        }
                    }
                    if(outSequenStarted){
                        inSequenStarted = false;
                        inout = false;
                        long outcurrentTime = System.currentTimeMillis() - startTimeout;
                        if(outcurrentTime < 1500){
                            shenshou = true;
                            lefthand.setPosition(0.35);
                            righthand.setPosition(1 - 0.35);
                        }else {
                            shenshou = false;
                            outSequenStarted = false;
                            inout = true;
                        }
                    }
                    if(inSequenStarted){
                        outSequenStarted = false;
                        inout = false;
                        long incurrentTime = System.currentTimeMillis() - startTimemin;
                        if(incurrentTime < 1500){
                            lefthand.setPosition(0);
                            righthand.setPosition(1);
                        }else {
                            inSequenStarted = false;
                            inout = true;
                        }
                    }
//                    lefthand.setPosition(handleRange((lefthand.getPosition() + ((gamepad2.left_trigger - gamepad2.right_trigger)*0.05)),0,0.3));
//                    righthand.setPosition(1 - (handleRange((lefthand.getPosition() + ((gamepad2.left_trigger - gamepad2.right_trigger)*0.05)),0,0.3)));
//                    lefthand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    telemetry.addData("p",(handleRange((lefthand.getPosition() + ((gamepad2.left_trigger - gamepad2.right_trigger)*0.05)),0,0.3)));
                    telemetry.addData("rp",righthand.getPosition());
                    telemetry.addData("lp",lefthand.getPosition());
//                    if(lefthand.getCurrentPosition() > 1600){
//                        lefthand.setPower(handleRange((gamepad2.left_trigger - gamepad2.right_trigger),-1,0));
//
//                    }
//                    else if(lefthand.getCurrentPosition() <= 10){
//                        lefthand.setPower(handleRange((gamepad2.left_trigger - gamepad2.right_trigger),0,1));
//                    }
//                    else {
//                        lefthand.setPower(handleRange(gamepad2.left_trigger - gamepad2.right_trigger,-1, 1));
//
//                    }
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
//                telemetry.addData("lefthand",lefthand.getCurrentPosition());
                telemetry.addData("gamepad", (gamepad2.left_trigger - gamepad2.right_trigger));

                telemetry.addData("Front Right Power", frontRightPower);
                telemetry.addData("Back Right Power", backRightPower);

                telemetry.addData("xuangua power",xuangua.getPower());
                telemetry.addData("xuangua position",xuanguacp);
                // change the background color to match the color detected by the RGB sensor.
                // pass a reference to the hue, saturation, and value array as an argument
                // to the HSVToColor method.
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
        }
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}







