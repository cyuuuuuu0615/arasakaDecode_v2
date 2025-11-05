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
public class shenshoujiadongxi extends LinearOpMode{

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
        DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor1");
        DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor2");
        DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor0");
        DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor3");
        DcMotorEx xuangua = (DcMotorEx) hardwareMap.dcMotor.get("motor5");
        DcMotorEx lefthand = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
        Servo shoubi = hardwareMap.get(Servo.class,"servo2");
        Servo shouzhang = hardwareMap.get(Servo.class,"servo3");
        Servo shouzhang0 = hardwareMap.get(Servo.class,"servo0");
        Servo shouzhang1 = hardwareMap.get(Servo.class,"servo1");

        lefthand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lefthand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        xuangua.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xuangua.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xuangua.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        if(opModeIsActive()){
            double handping = 0.3;
            double shuzhangop = 0.6;
            double shouzhanged = 0.3;
            double shuzhang0op = 0;
            double shouzhang0ed = 1;
            double shuzhang1op = 1;
            double shouzhang1ed = 0;
            int one = 100;
            int two = 233;
            int lastposition = 0;
            shoubi.setPosition(handping);
            shouzhang.setPosition(shouzhanged);
            while (opModeIsActive()){
                int xuanguacp= xuangua.getCurrentPosition();
                xuangua.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                if((gamepad1.left_trigger - gamepad1.right_trigger) != 0){
                    xuangua.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lastposition = xuanguacp;
                }
                if ((gamepad1.left_trigger - gamepad1.right_trigger) == 0){
                    xuangua.setPower(1);
                    xuangua.setTargetPosition(lastposition);
                    xuangua.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if(gamepad2.left_bumper){
                    shouzhang1.setPosition(shuzhang1op);
                    shouzhang0.setPosition(shuzhangop);

                } else if (gamepad2.right_bumper) {
                     shouzhang1.setPosition(shouzhang1ed);
                     shouzhang0.setPosition(shouzhanged);
                }
                telemetry.addData("xuangua position",xuanguacp);

                telemetry.update();
            }
        }

    }
}
