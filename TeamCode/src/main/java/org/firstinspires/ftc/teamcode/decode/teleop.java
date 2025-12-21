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
        DcMotor intakeMotor4 = hardwareMap.get(DcMotor.class, "motor4");
        Servo fasheServo = hardwareMap.get(Servo.class,"servo1");
        Servo baseServo = hardwareMap.get(Servo.class,"servo2");
        DcMotor shootingMotor = hardwareMap.get(DcMotor.class, "motor5");
        CRServo angleServo = hardwareMap.get(CRServo.class,"servo3");
        Servo zhaServo4 = hardwareMap.get(Servo.class,"servo4");
        Servo zhaServo5 = hardwareMap.get(Servo.class,"servo5");






        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor4.setDirection(DcMotorSimple.Direction.REVERSE);
        zhaServo5.setDirection(Servo.Direction.REVERSE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fasheServo.scaleRange(0, 0.5);
        baseServo.setPosition(0);




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

            if(gamepad1.dpad_up){
                intakeMotor4.setPower(-1);
            }
            if(gamepad1.dpad_down){
                intakeMotor4.setPower(0);
            }
            if(gamepad1.a){
                baseServo.setPosition(0.0471);
                zhaServo4.setPosition(1);
                zhaServo5.setPosition(1);
            }
            if(gamepad1.b){
                baseServo.setPosition(.4314);
                zhaServo4.setPosition(1);
                zhaServo5.setPosition(1);
            }
            if(gamepad1.x){
                baseServo.setPosition(.8196);
                zhaServo4.setPosition(1);
                zhaServo5.setPosition(1);
            }
            if(gamepad1.y){
                baseServo.setPosition(0);
                zhaServo4.setPosition(0);
                zhaServo5.setPosition(0);
            }
            if(gamepad1.dpad_left){
                baseServo.setPosition(.7137);
                zhaServo4.setPosition(0);
                zhaServo5.setPosition(0);
            }
            if(gamepad1.dpad_right){
                baseServo.setPosition(.3529);
                zhaServo4.setPosition(0);
                zhaServo5.setPosition(0);
            }
            if(gamepad1.left_bumper){
                fasheServo.setPosition(.8);
                sleep(300);
                fasheServo.setPosition(0);
            }

            shootingMotor.setPower(gamepad1.right_trigger);
//            if(gamepad1.left_bumper){
//                angleServo.setPower(.1);
//            }else if(gamepad1.right_bumper){
//                angleServo.setPower(-.1);
//            }else {
//                angleServo.setPower(0);
//            }




            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("intakeMotor4",intakeMotor4.getPower());
            telemetry.addData("shootingMotor",shootingMotor.getPower());
            telemetry.addData("angleServo",angleServo.getPower());
            telemetry.addData("fasheServo",fasheServo.getPosition());
            telemetry.addData("baseServo",baseServo.getPosition());


            telemetry.update();
        }



    }
}
