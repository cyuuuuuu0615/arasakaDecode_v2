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
public class chanziceshi extends LinearOpMode{

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
        Servo jiazixiaobi = hardwareMap.get(Servo.class,"servo2");
        Servo shoubi = hardwareMap.get(Servo.class,"servo6");
        Servo shouzhang = hardwareMap.get(Servo.class,"servo3");
        Servo xuanguaxiaobi = hardwareMap.get(Servo.class,"servo4");
        Servo xuanguadabi = hardwareMap.get(Servo.class,"servo5");
        Servo chanzi = hardwareMap.get(Servo.class,"servo7");


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
            double handping = 0.25;
            double handup = 0.78;
            double handdon = 0.1;
            double shuzhangop = 1;
            double shouzhanged = 0;
            double shoouzhangop = 1;
            double shuzhang0op = 0.54;
            double shouzhang0ed = 1;
            double shuzhang1op = 0.46;
            double shouzhang1ed = 0;
            double jiazixiaobidown = 0.7;
            double chanzidown = 0.45;
            double chanziup = 1;
            double handchanziup = 0.6104;
            double jiazixiaobichanziup = 0.2235;
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
            shouzhang.setPosition(shoouzhangop);
//            xuanguadabi.setPosition(xuanguadabidown);
//            xuanguaxiaobi.setPosition(xuanguaxiaobidown);
            jiazixiaobi.setPosition(jiazixiaobidown);
            chanzi.setPosition(chanzidown);
//            if((gamepad1.left_trigger - gamepad1.right_trigger) == 0){
//                telemetry.addData("fix position", xuangua.getCurrentPosition());
//            }
            while (opModeIsActive()){
                telemetry.addData("shoubi position", shoubi.getPosition());
                telemetry.addData("jiazixiaobi position", jiazixiaobi.getPosition());
                telemetry.addData("chanzi position",chanzi.getPosition());

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
                        jiazixiaobi.setPosition(jiazixiaobichanziup);
                        shoubi.setPosition(handchanziup);
                    } else if (currentTime < 2000) {
                        shouzhang.setPosition(shuzhangop);
                    } else if (currentTime < 2500) {
                        jiazixiaobi.setPosition(jiazixiaobidown);
                        shoubi.setPosition(handping);
                    } else {
                        isSequenceStarted = false;
                    }
                }
                if (gamepad2.right_bumper){
                    isSequenceStarted5 = true;
                    startTime5 = System.currentTimeMillis();
                }
                if (isSequenceStarted5){
                    long currentTime5 = System.currentTimeMillis() - startTime5;

                    if (currentTime5 < 1000){
                        chanzi.setPosition(chanziup);
                    }
                    else if(currentTime5 < 2000){
                        chanzi.setPosition(chanzidown);
                    }
                    else{
                        isSequenceStarted5 = false;
                    }
                }
                telemetry.update();
            }
        }

    }
}
