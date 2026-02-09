//package org.firstinspires.ftc.teamcode.decode;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import java.time.Instant;
//
//@TeleOp
//public class sampleTeleop extends LinearOpMode {
//
//
//
//
//    boolean reset = false;
//    boolean fasheOn = false;
//    boolean get = false;
//    boolean input = false;
//    boolean output = false;
//    long startTime1 = 0;
//    long currentTime1 = 0;
//    long startTime2 = 0;
//    long currentTime2 = 0;
//    long startTime3 = 0;
//    long currentTime3 = 0;
//    long startTime4 = 0;
//    long currentTime4 = 0;
//    long startTime5 = 0;
//    long currentTime5 = 0;
//    long startTime6 = 0;
//    long currentTime6 = 0;
//    long startTime7 = 0;
//    long currentTime7 = 0;
//
//
//    public void runOpMode(){
//        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
//        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
//        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
//        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
//        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
//
//
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//
//        waitForStart();
//
//
//
//
//
//        long startTime = System.currentTimeMillis();
//
//
//
//        while(opModeIsActive()){
//
//
//
//            double x = gamepad1.left_stick_x;
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double rx = gamepad1.right_stick_x;
//
//            double theta = Math.atan2(y, x);
//            double power = Math.hypot(x,y);
//
//            double sin = Math.sin(theta - Math.PI/4);
//            double cos = Math.cos(theta - Math.PI/4);
//            double max = Math.max(Math.abs(sin), Math.abs(cos));
//
//
//            double frontLeftPower = power * cos/max + rx;
//            double frontRightPower = power * sin/max - rx;
//            double backLeftPower = power * sin/max + rx;
//            double backRightPower = power * cos/max - rx;
//
//            if ((power + Math.abs(rx)) > 1){
//                frontLeftPower   /= power + Math.abs(rx);
//                frontRightPower /= power + Math.abs(rx);
//                backLeftPower    /= power + Math.abs(rx);
//                backRightPower  /= power + Math.abs(rx);
//            }
//
//
//            if(gamepad1.dpad_up){
//                intakeMotor.setPower(-1);
//            }
//            if(gamepad1.dpad_down){
//                intakeMotor.setPower(0);
//            }
//
//
//
//
//
//
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);
//
//            telemetry.addData("intakeMotor4",intakeMotor.getPower());
//            telemetry.update();
//        }
//
//
//
//    }
//}
