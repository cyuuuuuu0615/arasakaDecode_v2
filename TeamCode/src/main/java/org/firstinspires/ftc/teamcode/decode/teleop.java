package org.firstinspires.ftc.teamcode.decode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.time.Instant;

@TeleOp
public class teleop extends LinearOpMode {

    //ColorSensor color;

    public static double handlerange(double x,double a,double b){
        if(x>a){
            return a;
        }else if(x<b){
            return b;
        }else{
            return x;
        }
    }


    boolean dipanzhuan = false;
    boolean reset = false;
    boolean fasheOn = false;
    boolean get = false;
    boolean input = false;
    boolean output = false;
    long startTime1 = 0;
    long currentTime1 = 0;
    long startTime2 = 0;
    long currentTime2 = 0;
    long startTime3 = 0;
    long currentTime3 = 0;
    long startTime4 = 0;
    long currentTime4 = 0;
    long startTime5 = 0;
    long currentTime5 = 0;
    long startTime6 = 0;
    long currentTime6 = 0;
    long startTime7 = 0;
    long currentTime7 = 0;


    public void runOpMode(){
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
        DcMotor fasheMotor = hardwareMap.get(DcMotor.class, "motor5");
        CRServo fasheServo = hardwareMap.get(CRServo.class, "servo0");
//        DcMotor dipanmotor = hardwareMap.get(DcMotor.class, "motor4");
//        dipanmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        dipanmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        fasheMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        dipanmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();



//        int dplp = dipanmotor.getCurrentPosition();


        long startTime = System.currentTimeMillis();



        while(opModeIsActive()){



            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double rx = gamepad1.right_stick_x;

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



            if(gamepad1.left_bumper){
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }
            fasheMotor.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
//------------------------------------------------------------------------------------------

            if(fasheOn){
                currentTime3 = System.currentTimeMillis() - startTime3;
                fasheServo.setPower(-1);
                if(currentTime3>5000){
                    fasheOn = false;
                }
            }

            if(gamepad1.a){
                fasheOn = true;
                startTime3 = System.currentTimeMillis();
            }

            if(!fasheOn){
                fasheServo.setPower(0);
            }

//------------------------------------------------------------------------------------------

//            dipanmotor.setPower(gamepad1.right_trigger);
//
//            if(dipanzhuan) {
//                currentTime5 = System.currentTimeMillis() - startTime5;
//                dipanmotor.setTargetPosition(0);
//                dipanmotor.setPower(-1);
//                dipanmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                dplp = dipanmotor.getCurrentPosition();
//                if (currentTime5 > 2000) {
//                    dipanzhuan = false;
//                }
//            }
//
//
//            if(!dipanzhuan){
//                if(gamepad1.left_trigger + gamepad1.right_trigger == 0){
//                    dipanmotor.setTargetPosition(dplp);
//                    dipanmotor.setPower(1);
//                    dipanmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }else{
//                    dplp = dipanmotor.getCurrentPosition();
//                    dipanmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    dipanmotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
//                }
//            }




            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

//            telemetry.addData("dipanmotor",dipanmotor);
            telemetry.addData("fasheMotor",fasheMotor);
            telemetry.addData("intakeMotor",intakeMotor);

            telemetry.update();
        }









    }
}
