package org.firstinspires.ftc.teamcode.Test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/*
 * This OpMode demonstrates how to use a REV Robotics Touch Sensor, REV Robotics Magnetic Limit Switch, or other device
 * that implements the TouchSensor interface. Any touch sensor that connects its output to ground when pressed
 * (known as "active low") can be configured as a "REV Touch Sensor". This includes REV's Magnetic Limit Switch.
 *
 * The OpMode assumes that the touch sensor is configured with a name of "sensor_touch".
 *
 * A REV Robotics Touch Sensor must be configured on digital port number 1, 3, 5, or 7.
 * A Magnetic Limit Switch can be configured on any digital port.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Sensor: REV touch sensor", group = "Sensor")
public class jiazishoubi extends LinearOpMode {


//    TouchSensor touchSensor;  // Touch sensor Object
//    CRServo testservo;

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
    public void runOpMode() {

        // get a reference to our touchSensor object.

        DcMotorEx lefthand = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
        Servo shoubi = hardwareMap.get(Servo.class,"servo2");
//        Servo shouwan = hardwareMap.get(Servo.class,"servo1");
        Servo shouzhang = hardwareMap.get(Servo.class,"servo0");
        TouchSensor closer = hardwareMap.get(TouchSensor.class, "toucher1");
        TouchSensor opener = hardwareMap.get(TouchSensor.class,"toucher2");
        lefthand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // wait for the start button to be pressed.
        waitForStart();

        // while the OpMode is active, loop and read whether the sensor is being pressed.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        boolean result_arm = true;
//        boolean result_shouzhang = false;
        if(opModeIsActive()){
            double handup = 0.05;
            double handdon = 0.85;
            double handping = 0.45;
            double shuzhangop = 0.6;
            double shouzhanged = 0.3;
            double shouwanweizhi = 0.5;
            int one = 100;
            int two = 233;
//            shouwan.setPosition(shouwanweizhi);
            shoubi.setPosition(handping);
            shouzhang.setPosition(shouzhanged);

            while (opModeIsActive()) {


                // send the info back to driver station using telemetry function.
                boolean open = false;
                boolean close = false;
                //  move on(+) true move back(-) false
                shoubi.setPosition(handleRange(Math.abs(gamepad2.left_stick_y), 0.05, 0.85));

                if(closer.isPressed()){
//                lefthand.setPower(0);
                    result_arm = true;
                    telemetry.addLine("HANDIN");
                }

                if (opener.isPressed()){
//                lefthand.setPower(0);
                    result_arm = false;
                    telemetry.addLine("HANDOUT");
                }

//                if (result_arm){
//                    if(gamepad2.circle){
//
////                    lefthand.setPower(1);
////                    lefthand.setPower(1);
//
//
//                    }
//
//                }
//                else {
//                    if(gamepad2.circle){
//
////                    lefthand.setPower(-1);
////                    lefthand.setPower(-1);
//
//                    }
//
//                }
                if (gamepad2.triangle){
                    shouzhang.setPosition(shuzhangop);
                }
                if (gamepad2.square){
                    shouzhang.setPosition(shouzhanged);
                }



                if (gamepad2.right_bumper){

                    lefthand.setTargetPosition(0);
                    lefthand.setPower(-0.5);
                    lefthand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
//                if (lefthand.getCurrentPosition() == 0){
//                    lefthand.setTargetPosition(one);
//                    lefthand.setPower(0.5);
//                    lefthand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//                else if(lefthand.getCurrentPosition() == one){
//                    lefthand.setTargetPosition(two);
//                    lefthand.setPower(0.5);
//                    lefthand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                } else if (lefthand.getCurrentPosition() == two) {
//                    lefthand.setTargetPosition(one);
//                    lefthand.setPower(-0.5);
//                    lefthand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
                if (gamepad2.circle){
                    lefthand.setTargetPosition(one);
                    lefthand.setPower(0.5);
                    lefthand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if(gamepad2.cross){
                    lefthand.setTargetPosition(two);
                    lefthand.setPower(0.5);
                    lefthand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }


                //            if (touchSensor.isPressed()) {
//                telemetry.addData("Touch Sensor", "Is Pressed");
//                testservo.setPower(0);
//            } else {
//                telemetry.addData("Touch Sensor", "Is Not Pressed");
//                testservo.setPower(1);
//            }

                telemetry.update();
            }

        }

    }
}
