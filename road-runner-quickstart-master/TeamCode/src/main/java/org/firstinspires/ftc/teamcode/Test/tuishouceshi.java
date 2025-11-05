package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class tuishouceshi extends LinearOpMode{

    public static double handleRange(double number, double start, double end) {
        if (number < start) {
            return start;
        } else if (number > end) {
            return end;
        } else {
            return number;
        }
    }

    public static boolean inRange(double number, double start, double end) {
        return number >= start && number <= end;
    }

    @Override
    public void runOpMode(){
        DcMotorEx lefthand = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
        Servo shoubi = hardwareMap.get(Servo.class,"servo2");
//        Servo shouwan = hardwareMap.get(Servo.class,"servo1");
        Servo shouzhang = hardwareMap.get(Servo.class,"servo0");
        lefthand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lefthand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lefthand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lefthand.setDirection(DcMotorSimple.Direction.REVERSE);
        // wait for the start button to be pressed.
        waitForStart();

        // while the OpMode is active, loop and read whether the sensor is being pressed.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        boolean result_arm = true;
        boolean result_shouzhang = false;
        if(opModeIsActive()){
            double handup = 0.05;
            double handdon = 0.85;
            double handping = 0.45;
            double shuzhangop = 0.6;
            double shouzhanged = 0.3;
            double shouwanweizhi = 0.5;
            int left_hand_position = 0;
            int one = 100;
            int two = 233;
            int error = 55;
//            shouwan.setPosition(shouwanweizhi);
            shoubi.setPosition(handping);
            shouzhang.setPosition(shouzhanged);
            while (opModeIsActive()) {
                if(lefthand.getCurrentPosition() <= 10){
                    lefthand.setPower(handleRange((-(gamepad2.left_trigger - gamepad2.right_trigger)),0,0.6));
                }
                else {
                    lefthand.setPower(handleRange((-(gamepad2.left_trigger - gamepad2.right_trigger)),-0.6, 0.6));

                }
                telemetry.addData("Current Position",lefthand.getCurrentPosition());
                telemetry.update();
            }

        }


    }

}
