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
public class xuanguaceshi extends LinearOpMode{

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

    public void runOpMode(){
//        DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor1");
//        DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor2");
//        DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor0");
//        DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor3");
//        DcMotorEx xuangua = (DcMotorEx) hardwareMap.dcMotor.get("motor5");
//        DcMotorEx xuangua2 = (DcMotorEx) hardwareMap.dcMotor.get("motor6");
        Servo shouzhang0 = hardwareMap.get(Servo.class,"servo0");
        Servo shouzhang1 = hardwareMap.get(Servo.class,"servo1");
////        DcMotorEx lefthand = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
        Servo shoubi = hardwareMap.get(Servo.class,"servo6");
        Servo shouzhang = hardwareMap.get(Servo.class,"servo3");
        Servo xuanguaxiaobi = hardwareMap.get(Servo.class,"servo4");
        Servo xuanguadabi = hardwareMap.get(Servo.class,"servo5");


//        xuangua.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        xuangua.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        xuangua.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        xuangua1.setDirection(DcMotorSimple.Direction.REVERSE);
        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        if(opModeIsActive()){
            double handping = 0.43;
            double handup = 0;
            double handdon = 1;
            double shuzhangop = 1;
            double shouzhanged = 0;
            double shoouzhangop = 1;
            double shuzhang0op = 0.54;
            double shouzhang0ed = 1;
            double shuzhang1op = 0.46;
            double shouzhang1ed = 0;
            int one = 100;
            int two = 233;
            boolean isSequenceStarted = false;
            boolean isSequenceStarted1 = false;
            boolean isSequenceStarted2 = false;
            boolean isSequenceStarted3 = false;
            boolean isSequenceStarted4 = false;
            boolean isSequenceStarted5 = false;
            boolean isSequenceStarted6 = false;
            long startTime = 0;
            long startTime1 = 0;
            long startTime2 = 0;
            long startTime3 = 0;
            long startTime4 = 0;
            long startTime5 = 0;
            long startTime6 = 0;
//            boolean updown = false;
            double xuanguadabidown = 1;
            double xuanguadabiup = 0.3;
            double xuanguaxiaobidown = 0;
            double xuanguaxiaobiup = 1;
            shoubi.setPosition(handping);
            shouzhang.setPosition(shouzhanged);
            xuanguadabi.setPosition(xuanguadabidown);
            xuanguaxiaobi.setPosition(xuanguaxiaobidown);
//            if((gamepad1.left_trigger - gamepad1.right_trigger) == 0){
//                telemetry.addData("fix position", xuangua.getCurrentPosition());
//            }
            while (opModeIsActive()){
//                telemetry.addData("xuangua power",xuangua.getPower());
//                shoubi.setPosition(handup);
//                xuanguaxiaobi.setPosition(gamepad2.left_trigger - gamepad2.right_trigger);
//                xuanguadabi.setPosition(handleRange(Math.abs(gamepad2.left_stick_y),0.3,1));

//                telemetry.addData("xuangua position",xuangua.getCurrentPosition());
                telemetry.addData("xuanguaxiaobi position", xuanguaxiaobi.getPosition());
                telemetry.addData("xuanguadabi position",xuanguadabi.getPosition());

//                if(lefthand.getCurrentPosition() > 80){
//                    lefthand.setPower(handleRange((gamepad2.left_trigger - gamepad2.right_trigger),-0.6,0));
//
//                }
//                else if(lefthand.getCurrentPosition() <= 25){
//                    lefthand.setPower(handleRange((gamepad2.left_trigger - gamepad2.right_trigger),0,0.6));
//                }
//                else {
//                    lefthand.setPower(handleRange(gamepad2.left_trigger - gamepad2.right_trigger,-0.6, 0.6));
//
//                }
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
                if (gamepad2.left_bumper && !isSequenceStarted3) { //shuangbiguan shouzhangkai
                    isSequenceStarted3 = true;
                    startTime3 = System.currentTimeMillis();
                }
                if (isSequenceStarted3) {
                    long currentTime3 = System.currentTimeMillis() - startTime3;

                    if (currentTime3 < 500) {
                        shouzhang1.setPosition(shouzhang1ed);
                        shouzhang0.setPosition(shouzhang0ed);
                    }
                    else if (currentTime3 < 1000) {
                        shouzhang.setPosition(shoouzhangop);
                    }
                    else if (currentTime3 < 1500) {
                        xuanguadabi.setPosition(xuanguadabiup);
                        xuanguaxiaobi.setPosition(xuanguaxiaobiup);
                    }
                    else {
                        isSequenceStarted3 = false;
                    }
                }
                if (gamepad2.right_bumper){ //jiaziguan shuangbikai
                    isSequenceStarted6 = true;
                    startTime6 = System.currentTimeMillis();
                }
                if(isSequenceStarted6){
                    long currentTime6 = System.currentTimeMillis() - startTime6;
                    if(currentTime6 < 500){
                        shouzhang1.setPosition(shuzhang1op);
                        shouzhang0.setPosition(shuzhang0op);
                    }
                    else if(currentTime6 < 1000){
                        xuanguadabi.setPosition(xuanguadabidown);
                        xuanguaxiaobi.setPosition(xuanguadabidown);
                    }
                    else {
                        isSequenceStarted6 = false;
                    }
                }

//                if (gamepad2.circle){
//                    isSequenceStarted5 = true;
//                    startTime5 = System.currentTimeMillis();
//                }
//                if (isSequenceStarted5) {
//                    long currentTime5 = System.currentTimeMillis() - startTime5;
//
//                    if (currentTime5 < 500) {
//                        shouzhang0.setPosition(shouzhang0ed);
//                        shouzhang1.setPosition(shouzhang1ed);
//                    }
//                    else if (currentTime5 < 1000) {
//                        shouzhang.setPosition(shoouzhangop);
//                    }
//                    else if (currentTime5 < 1500) {
//                        updown = true;
//                    } else if (currentTime5 < 2000) {
//                        shouzhang0.setPosition(shuzhang0op);
//                        shouzhang1.setPosition(shuzhang1op);
//                    }
//                    else {
//                        isSequenceStarted5 = false;
//                    }
//                }
//                if(gamepad2.cross){
//                    shouzhang0.setPosition(shuzhang0op);
//                    shouzhang1.setPosition(shuzhang1op);
//                    xuanguadabi.setPosition(xuanguaxiaobidown);
//                    xuangua.setTargetPosition(10);
//                    xuangua.setPower(-1);
//                    xuangua.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    updown = false;
//                }
//                if(gamepad2.circle){
//                    updown = true;
//                }
//                if (gamepad2.cross){
//                    updown = false;
//                }
//                if (updown){
//                    xuangua.setTargetPosition(3000);
//                    xuangua.setPower(1);
//                    xuangua.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//                if (!updown){
//                    xuangua.setTargetPosition(50);
//                    xuangua.setPower(-1);
//                    xuangua.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
                telemetry.update();
            }
        }

    }
}
