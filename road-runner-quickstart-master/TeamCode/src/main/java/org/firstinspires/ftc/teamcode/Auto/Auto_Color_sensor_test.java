package org.firstinspires.ftc.teamcode.Auto;
// RR-specific imports
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp.M;

import com.acmerobotics.roadrunner.ParallelAction;

import java.util.List;

@Config
@Autonomous
public class Auto_Color_sensor_test extends LinearOpMode{   //60cm:25unit          1 cm: 5/12 unit
    double lefthandin = 0;
    double lefthandout = 1;
    double righthandin = 1 - lefthandin;
    double righthandout = 1 - lefthandout;
    double shuzhangop = 1;
    double shouzhanged = 0;
    double shuzhang0op = 0;
    double shouzhang0ed = 1;
    double shuzhang1op = 1;
    double shouzhang1ed = 0;
    double zuoshouhandup = 0;
    double zuoshouhanddon = 0.5553;
    double zuoshouhandping = 0.4;
    double youshouhandup = 1 - zuoshouhandup;
    double youshouhanddon = 1 - zuoshouhanddon;
    double youshouhandping = 1 - zuoshouhandping;
    double chanziping = 0;
    double chanzidown = 0.45;
    double chanziup = 1;
    double handchanziup = 0.6104;
    double jiazixiaobichanziup = 0;
    double jiazixiaobidown = 1;
    boolean isSequenceStarted = false;
    boolean isSequenceStarted1 = false;
    boolean isSequenceStarted5 = false;
    long startTime = 0;
    long startTime1 = 0;
    long startTime5 = 0;
    int lefthandlastposition = 5;
    int one = 100;
    int two = 233;
    int lastxuanguaposition = 0;
    boolean mode = false;
    boolean jiazizheng = false;
    boolean jiazifan = false;
    boolean yellow = true;
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
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(50 - 25 - 25,75 - 25 - 50,Math.PI*1.5);
        Pose2d secondPose = new Pose2d(43.125 - 25,45 - 50,Math.PI*1.35);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor1");
        DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor2");
        DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor0");
        DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor3");
        DcMotorEx xuangua = (DcMotorEx) hardwareMap.dcMotor.get("motor5");
        Servo zuoshoubi = hardwareMap.get(Servo.class,"servo6");
        Servo shouzhang0 = hardwareMap.get(Servo.class,"servo4");
        Servo shouzhang1 = hardwareMap.get(Servo.class,"servo1");
        Servo youshoubi = hardwareMap.get(Servo.class,"servo11");
        Servo lefthand = hardwareMap.get(Servo.class,"servo4");
        Servo righthand = hardwareMap.get(Servo.class,"servo5");
        Servo chanzi = hardwareMap.get(Servo.class,"servo7");

//        lefthand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lefthand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lefthand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xuangua.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xuangua.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xuangua.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        xuangua.setDirection(DcMotorSimple.Direction.REVERSE);
//        lefthand.setDirection(DcMotorSimple.Direction.REVERSE);

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


        shouzhang1.setPosition(shuzhang1op);
        shouzhang0.setPosition(shuzhang0op);
        chanzi.setPosition(chanziping);
        zuoshoubi.setPosition(zuoshouhandping);
        youshoubi.setPosition(youshouhandping);
        lefthand.setPosition(lefthandin);
        righthand.setPosition(righthandin);
//        Actions.runBlocking(new SequentialAction(drive.actionBuilder(beginPose)
//                .splineTo(new Vector2d(35.5 - 25,-16),Math.PI*1.5)
//                .stopAndAdd(new Auto.zhengshouGetSpec())
//                .setReversed(true)
//                .splineTo(new Vector2d(40 - 25,50 - 55),-Math.PI*1.75)
//                .stopAndAdd(new ParallelAction(
//                        new Auto.shenshoufangdongxi()
//                ))
//                .setReversed(false)
//                .splineTo(new Vector2d(35.5 - 25 + 12 - 8*5/12,-16.5),Math.PI*1.5)
//                .stopAndAdd(new Auto.zhengshouGetSpec())
//                .setReversed(true)
//                .splineTo(new Vector2d(40.5 - 25,50.5 - 55),-Math.PI*1.75)
//                .stopAndAdd(new ParallelAction(
//                        new Auto.shenshoufangdongxi()
//
//                ))
//                .setReversed(false)
//                .splineTo(new Vector2d(10,-38),Math.PI/15)
//                .stopAndAdd(new Auto.zhengshouGetSpec())
//                .setReversed(true)
//                .splineTo(new Vector2d(41.5 - 25,50 - 55),-Math.PI*1.75)
//                .stopAndAdd(new ParallelAction(
//                        new Auto.shenshoufangdongxi()
//
//                ))
//                .build())
//        );
    }
    public class shenshoufangdongxi implements Action{

        DcMotorEx xuangua = (DcMotorEx) hardwareMap.dcMotor.get("motor5");
        Servo zuoshoubi = hardwareMap.get(Servo.class,"servo6");
        Servo youshoubi = hardwareMap.get(Servo.class,"servo11");
        Servo chanzi = hardwareMap.get(Servo.class,"servo7");

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            xuangua.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            xuangua.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            xuangua.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            xuangua.setDirection(DcMotorSimple.Direction.REVERSE);
            long startTimeUp = System.currentTimeMillis();
            zuoshoubi.setPosition(zuoshouhandping);
            youshoubi.setPosition(youshouhandping);
            while (true){
                long currentTimeUP = System.currentTimeMillis() - startTimeUp;
                if(currentTimeUP < 3000){
                    xuangua.setTargetPosition(6250);
                    xuangua.setPower(1);
                    xuangua.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (currentTimeUP < 6000) {
                    xuangua.setTargetPosition(10);
                    xuangua.setPower(1);
                    xuangua.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }else {
                    return false;
                }
                if((currentTimeUP > 2000)&&(currentTimeUP < 3500)){
                    chanzi.setPosition(chanziup);
                }else {
                    chanzi.setPosition(chanziping);
                }
            }
        }
    }
    public class shoudongceshi implements Action{
        DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor1");
        DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor2");
        DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor0");
        DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("motor3");
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while(true){
                double x = gamepad2.left_stick_x;
                double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
                //double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad2.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y + x - rx) / denominator;
//            double backRightPower = (y - x - rx) / denominator;
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
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                if(gamepad2.cross){
                    return false;
                }
            }
        }
    }




    public class zhengshouGetSpec implements Action{
        boolean get = false;
        public ColorSensor colorSensor;
        Servo jiazixiaobi = hardwareMap.get(Servo.class,"servo2");
        Servo shoubi = hardwareMap.get(Servo.class,"servo6");
        Servo shouzhang = hardwareMap.get(Servo.class,"servo3");
        Servo chanzi = hardwareMap.get(Servo.class,"servo7");
        Servo shouzhang0 = hardwareMap.get(Servo.class,"servo4");
        Servo shouzhang1 = hardwareMap.get(Servo.class,"servo1");
        Servo lefthand = hardwareMap.get(Servo.class,"servo0");
        Servo righthand = hardwareMap.get(Servo.class,"servo5");
        Servo zuoshoubi = hardwareMap.get(Servo.class,"servo6");
        Servo youshoubi = hardwareMap.get(Servo.class,"servo11");
        //        DcMotorEx lefthand = (DcMotorEx) hardwareMap.dcMotor.get("motor4");
        CRServo zhuazi0 = hardwareMap.get(CRServo.class,"servo8");
        CRServo zhuazi1 = hardwareMap.get(CRServo.class,"servo9");

        @Override
        public  boolean run(@NonNull TelemetryPacket telemetryPacket){
            boolean action = true;
            zuoshoubi.setPosition(zuoshouhanddon);
            youshoubi.setPosition(youshouhanddon);
            float hsvValues[] = {0F,0F,0F};

            // values is a reference to the hsvValues array.
            final float values[] = hsvValues;

            // get a reference to the RelativeLayout so we can change the background
            // color of the Robot Controller app to match the hue detected by the RGB sensor.
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

            // bPrevState and bCurrState represent the previous and current state of the button.
            boolean bPrevState = false;
            boolean bCurrState = false;

            // bLedOn represents the state of the LED.
            boolean bLedOn = true;
//__________________________________________________________________________
            // get a reference to our ColorSensor object.
            colorSensor = hardwareMap.get(ColorSensor.class, "I2C1");

            // Set the LED in the beginning
            colorSensor.enableLed(bLedOn);
            long startTimeget = System.currentTimeMillis();
            while (action){
                if (bCurrState && (bCurrState != bPrevState)) {

                    // button is transitioning to a pressed state. So Toggle LED
                    bLedOn = !bLedOn;
                    colorSensor.enableLed(bLedOn);
                }

                // update previous state variable.
                bPrevState = bCurrState;

                // convert the RGB values to HSV values.
                Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

                // send the info back to driver station using telemetry function.
                telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Clear", colorSensor.alpha());
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());
                telemetry.addData("Hue", hsvValues[0]);

                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });

                telemetry.update();
                if(!get){
                    startTime = System.currentTimeMillis();
                    if((startTime - startTimeget)%1000 == 0){
                        lefthand.setPosition(lefthand.getPosition() + 0.01);
                        righthand.setPosition(1 - lefthand.getPosition());
                    }
                    if(colorSensor.red() <= 600){
                        lefthand.setPosition(lefthandout);
                        righthand.setPosition(righthandout);
//                    if (!inRange(colorSensor.red(), 600, 1300)) {
                        zhuazi1.setPower(1);
                        zhuazi0.setPower(-1);
                        startTime = System.currentTimeMillis();
                    } else {
                        zhuazi0.setPower(0);
                        zhuazi1.setPower(0);
                        get = true;
                    }
                }else {
                    isSequenceStarted = true;
                    if(isSequenceStarted){
                        lefthand.setPosition(lefthandin);
                        righthand.setPosition(righthandin);
                        long currentTime = System.currentTimeMillis() - startTime;
                        if(currentTime < 500 + 500){
                            zuoshoubi.setPosition(zuoshouhandup);
                            youshoubi.setPosition(youshouhandup);
                        } else if (currentTime < 1500 + 500) {
                            zhuazi1.setPower(-1);
                            zhuazi0.setPower(1);
                        } else {
                            isSequenceStarted = false;
                            action = false;
                        }
                    }
                }
            }


            return false;
        }

    }


}
