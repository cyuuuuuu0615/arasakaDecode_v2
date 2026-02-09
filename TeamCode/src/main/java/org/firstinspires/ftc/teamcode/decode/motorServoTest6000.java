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
//public class motorServoTest6000 extends LinearOpMode {
//
//
//
//
//    boolean reset = false;
//    boolean fasheOn = false;
//    boolean get = false;
//    boolean input = false;
//    float output = 0;
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
////        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
////        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
////        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
////        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
//        DcMotor shootingMotor = hardwareMap.get(DcMotor.class, "motor0");
//        CRServo shangxiaAngle = hardwareMap.get(CRServo.class,"servo0");
//                shootingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//
//
////        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
////        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
////        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
////            double x = gamepad1.left_stick_x;
////            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
////            double rx = gamepad1.right_stick_x;
////
////            double theta = Math.atan2(y, x);
////            double power = Math.hypot(x,y);
////
////            double sin = Math.sin(theta - Math.PI/4);
////            double cos = Math.cos(theta - Math.PI/4);
////            double max = Math.max(Math.abs(sin), Math.abs(cos));
////
////
////            double frontLeftPower = power * cos/max + rx;
////            double frontRightPower = power * sin/max - rx;
////            double backLeftPower = power * sin/max + rx;
////            double backRightPower = power * cos/max - rx;
////
////            if ((power + Math.abs(rx)) > 1){
////                frontLeftPower   /= power + Math.abs(rx);
////                frontRightPower /= power + Math.abs(rx);
////                backLeftPower    /= power + Math.abs(rx);
////                backRightPower  /= power + Math.abs(rx);
////            }
////
//
//
////            if(gamepad1.dpad_up){
////                output = (float) (output - 0.1);
////            }
////            if(gamepad1.dpad_down){
////                output = (float) (output + 0.1);
////            }
//
//            shootingMotor.setPower(gamepad1.right_trigger);
//
//
//            if(gamepad1.left_bumper){
//                shangxiaAngle.setPower(.1);
//            }else if(gamepad1.right_bumper){
//                shangxiaAngle.setPower(-.1);
//            }else {
//                shangxiaAngle.setPower(0);
//            }
//
//
//
//
////
////            frontLeftMotor.setPower(frontLeftPower);
////            backLeftMotor.setPower(backLeftPower);
////            frontRightMotor.setPower(frontRightPower);
////            backRightMotor.setPower(backRightPower);
//
//            telemetry.addData("fashe Motor",shootingMotor.getPower());
//            telemetry.addData("Angle servo",shangxiaAngle.getPower());
//            telemetry.update();
//        }
//
//
//
//    }
//}
