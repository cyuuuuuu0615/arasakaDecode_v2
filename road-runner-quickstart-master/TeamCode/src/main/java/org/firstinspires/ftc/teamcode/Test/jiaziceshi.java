package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class jiaziceshi extends LinearOpMode{
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
    public void runOpMode() {
//        DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor1");
//        DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor2");
//        DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor0");
//        DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor3");
        DcMotorEx lefthand = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
//        DcMotorEx xuangua1 = (DcMotorEx) hardwareMap.dcMotor.get("motor5");
//        DcMotorEx xuangua2 = (DcMotorEx) hardwareMap.dcMotor.get("motor6");
        Servo shoubi = hardwareMap.get(Servo.class,"servo6");
        Servo jiazixiaobi = hardwareMap.get(Servo.class,"servo2");
        Servo shouzhang = hardwareMap.get(Servo.class,"servo3");
        Servo shouzhang0 = hardwareMap.get(Servo.class,"servo0");
        Servo shouzhang1 = hardwareMap.get(Servo.class,"servo1");

        lefthand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lefthand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lefthand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        xuangua1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        xuangua1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        xuangua1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);az
//        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        xuangua1.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if(opModeIsActive()){
            double shoouzhangop = 1;
            double shouzhanged = 0;
            double handping = 0.15;
            double handup = 0.78;
            double handdon = 0.1;
            double shuzhang0op = 0;
            double shouzhang0ed = 1;
            double shuzhang1op = 1;
            double shouzhang1ed = 0;
            shoubi.setPosition(handping);
            shouzhang.setPosition(shoouzhangop);
            boolean isSequenceStarted = false;
            boolean isSequenceStarted1 = false;
            long startTime = 0;
            long startTime1 = 0;
            while (opModeIsActive()){
                telemetry.addData("shoubi position",shoubi.getPosition());
                if(lefthand.getCurrentPosition() > 80){
                    lefthand.setPower(handleRange((gamepad2.left_trigger - gamepad2.right_trigger),-0.6,0));

                }
                else if(lefthand.getCurrentPosition() <= 25){
                    lefthand.setPower(handleRange((gamepad2.left_trigger - gamepad2.right_trigger),0,0.6));
                }
                else {
                    lefthand.setPower(handleRange(gamepad2.left_trigger - gamepad2.right_trigger,-0.6, 0.6));

                }

                if (gamepad2.triangle && !isSequenceStarted) {
                    isSequenceStarted = true;
                    startTime = System.currentTimeMillis();
                }
                if (isSequenceStarted) {
                    long currentTime = System.currentTimeMillis() - startTime;

                    if (currentTime < 500) {
                        shoubi.setPosition(handdon);
                    }
                    else if (currentTime < 1000) {
                        shouzhang.setPosition(shouzhanged);
                    }
                    else if (currentTime < 1500) {
                        shoubi.setPosition(handup);
                    }
                    else {
                        isSequenceStarted = false;
                    }
                }



                if(gamepad2.right_bumper){
                    shouzhang1.setPosition(shuzhang1op);
                    shouzhang0.setPosition(shuzhang0op);
                }
                if (gamepad2.left_bumper){
                    shouzhang1.setPosition(shouzhang1ed);
                    shouzhang0.setPosition(shouzhang0ed);
                }
                if(gamepad2.square){
//                    shoubi.setPosition(handping);
//                    shouzhang.setPosition(shoouzhangop);
                    isSequenceStarted1 = true;
                    startTime1 = System.currentTimeMillis();

                }
                if (isSequenceStarted1) {
                    long currentTime1 = System.currentTimeMillis() - startTime1;

                    if (currentTime1 < 500) {
                        shoubi.setPosition(handping);
                    }
                    else if (currentTime1 < 1000) {
                        shouzhang.setPosition(shoouzhangop);
                    }
                    else {
                        isSequenceStarted1 = false;
                    }
                }

                telemetry.addData("shouzhang postion",shouzhang.getPosition());
                telemetry.addData("lefthand position", lefthand.getCurrentPosition());


                telemetry.update();
            }
        }


    }
}
