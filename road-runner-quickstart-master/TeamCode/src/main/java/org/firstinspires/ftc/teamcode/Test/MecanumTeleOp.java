package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
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
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor1");
        DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor2");
        DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor0");
        DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor3");
        DcMotorEx lefthand = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
        DcMotorEx xuangua = (DcMotorEx) hardwareMap.dcMotor.get("motor5");
        DcMotorEx shoubi = (DcMotorEx) hardwareMap.dcMotor.get("motor6");
        Servo jiazixiaobi = hardwareMap.get(Servo.class,"servo2");
//        Servo shoubi = hardwareMap.get(Servo.class,"servo6");
        Servo shouzhang = hardwareMap.get(Servo.class,"servo3");
        Servo chanzi = hardwareMap.get(Servo.class,"servo7");
//        Servo shouzhang0 = hardwareMap.get(Servo.class,"servo0");
//        Servo shouzhang1 = hardwareMap.get(Servo.class,"servo1");

        lefthand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lefthand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lefthand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoubi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoubi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoubi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            double shuzhangop = 1;
            double shouzhanged = 0;
            double handping = 0.3;
            double handup = 0.78;
            double handdon = 0.1;
            double shuzhang0op = 0;
            double shouzhang0ed = 1;
            double shuzhang1op = 1;
            double shouzhang1ed = 0;
            double chanziping = 0.4778;
            double chanzidown = 0.45;
            double chanziup = 1;
            double handchanziup = 0.6104;
            double jiazixiaobichanziup = 0.2235;
            double jiazixiaobidown = 0.7;
            boolean isSequenceStarted = false;
            boolean isSequenceStarted1 = false;
            boolean isSequenceStarted5 = false;
            long startTime = 0;
            long startTime1 = 0;
            long startTime5 = 0;
            int lefthandlastposition = 5;
            int lp = 0;
            int one = 100;
            int two = 233;
            boolean updown = false;
//            shoubi.setPosition(handping);
            shouzhang.setPosition(shuzhangop);
            jiazixiaobi.setPosition(jiazixiaobidown);
            chanzi.setPosition(chanziping);
//            shouzhang0.setPosition(shouzhang0ed);
//            shouzhang0.setPosition(shouzhang0ed);
            while (opModeIsActive()) {
                int xuanguacp= xuangua.getCurrentPosition();


//                shoubi.setPosition(handleRange(gamepad2.left_stick_y + handping, 0.05, 0.85));
//                if (gamepad2.triangle && !isSequenceStarted) { //lianggeshouzhang
//                    isSequenceStarted = true;
//                    startTime = System.currentTimeMillis();
//                }
//                if (isSequenceStarted) {
//                    long currentTime = System.currentTimeMillis() - startTime;
//
//                    if (currentTime < 500) {
//                        shoubi.setPosition(handdon);
//                    }
//                    else if (currentTime < 1000) {
//                        shouzhang.setPosition(shouzhanged);
//                    }
//                    else if (currentTime < 1500) {
//                        shoubi.setPosition(handup);
//                    }
//                    else {
//                        isSequenceStarted = false;
//                    }
//                }
//
//                if(gamepad2.square){       //lianggeshouzhang
////                    shoubi.setPosition(handping);
////                    shouzhang.setPosition(shoouzhangop);
//                    isSequenceStarted1 = true;
//                    startTime1 = System.currentTimeMillis();
//
//                }
//                if (isSequenceStarted1) {
//                    long currentTime1 = System.currentTimeMillis() - startTime1;
//
//                    if (currentTime1 < 500) {
//                        shoubi.setPosition(handping);
//                    }
//                    else if (currentTime1 < 1000) {
//                        shouzhang.setPosition(shoouzhangop);
//                    }
//                    else {
//                        isSequenceStarted1 = false;
//                    }
//                }
//
//                if (gamepad2.circle){
//                    updown = true;
////                    isSequenceStarted2 = true;
////                    startTime2 = System.currentTimeMillis();
//                }
//
//                if (gamepad2.cross){
//                    updown = false;
//                }
                if (updown){
                    xuangua.setTargetPosition(3000);
                    xuangua.setPower(1);
                    xuangua.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (!updown){
                    xuangua.setTargetPosition(15);
                    xuangua.setPower(-1);
                    xuangua.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if(gamepad2.cross){
//                    shoubi.setPosition(handping);
                    shouzhang.setPosition(shuzhangop);
                }

                if (gamepad2.triangle && !isSequenceStarted) {
                    isSequenceStarted = true;
                    startTime = System.currentTimeMillis();
                }
                if (isSequenceStarted) {
                    long currentTime = System.currentTimeMillis() - startTime;

                    if (currentTime < 500) {
//                        shoubi.setPosition(handdon);
                    }
                    else if (currentTime < 1000) {
                        shouzhang.setPosition(shouzhanged);
                    }
                    else if (currentTime < 1500) {
//                        shoubi.setPosition(handchanziup);
                    }
                    else {
                        isSequenceStarted = false;
                    }
                }
                if (gamepad2.square){
                    isSequenceStarted5 = true;
                    startTime5 = System.currentTimeMillis();
                }
                if (isSequenceStarted5){
                    long currentTime5 = System.currentTimeMillis() - startTime5;
                    if(currentTime5 > 2000){
                        updown = true;
                    }
                    if(currentTime5 < 500){
                        jiazixiaobi.setPosition(jiazixiaobichanziup);
                    }
                    else if(currentTime5 < 1000) {
                        shouzhang.setPosition(shuzhangop);
                    } else if (currentTime5 < 1500) {
                        jiazixiaobi.setPosition(jiazixiaobidown);
//                        shoubi.setPosition(handping);
                    }
                    else if ((currentTime5 > 3500) && (currentTime5 < 4500)){
                        chanzi.setPosition(chanziup);
                    }
                    else if(currentTime5 < 5500){
                        chanzi.setPosition(chanziping);
                    }
                    else{
                        isSequenceStarted5 = false;
                        updown = false;
                    }
                }

                if(lefthand.getCurrentPosition() > 80){
                    lefthand.setPower(handleRange((gamepad2.left_trigger - gamepad2.right_trigger),-0.6,0));

                }
                else if(lefthand.getCurrentPosition() <= 10){
                    lefthand.setPower(handleRange((gamepad2.left_trigger - gamepad2.right_trigger),0,0.6));
                }
                else {
                    lefthand.setPower(handleRange(gamepad2.left_trigger - gamepad2.right_trigger,-0.6, 0.6));

                }
                if((gamepad1.left_trigger - gamepad1.right_trigger) != 0) {
                    shoubi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lp = shoubi.getCurrentPosition();
                    shoubi.setPower(handleRange(gamepad1.left_trigger - gamepad1.right_trigger, -0.2, 0.2));
                }
                else{
                    shoubi.setTargetPosition(lp);
                    shoubi.setPower(0.2);
                    shoubi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if((gamepad2.left_trigger - gamepad2.right_trigger) != 0){
                    lefthand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lefthandlastposition = lefthand.getCurrentPosition();
                }else {
                    lefthand.setPower(1);
                    lefthand.setTargetPosition(lefthandlastposition);
                    lefthand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
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


                telemetry.update();
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }

        }


    }
}
