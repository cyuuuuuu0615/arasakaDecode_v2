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
public class xuangua extends LinearOpMode{

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
//        DcMotorEx lefthand = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
        Servo shoubi = hardwareMap.get(Servo.class,"servo2");
        Servo shouzhang = hardwareMap.get(Servo.class,"servo0");
        Servo chanzi = hardwareMap.get(Servo.class,"servo7");
        Servo saodijiqiren  = hardwareMap.get(Servo.class,"servo3");

//        lefthand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lefthand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        xuangua.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xuangua.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xuangua.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //xuangua.setDirection(DcMotorSimple.Direction.REVERSE);
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
            double handping = 0.45;
            double shuzhangop = 0.6;
            double shouzhanged = 0.3;
            int one = 100;
            int two = 233;
            shoubi.setPosition(handping);
            shouzhang.setPosition(shouzhanged);
            int xuanguaop = xuangua.getCurrentPosition();
            while (opModeIsActive()){
                int xuanguacp= xuangua.getCurrentPosition() - xuanguaop;
                chanzi.setPosition(gamepad1.left_stick_y);

                telemetry.addData("xuangua position",chanzi.getPosition());

                telemetry.update();
            }
        }

    }
}
