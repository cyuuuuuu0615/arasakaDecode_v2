package org.firstinspires.ftc.teamcode.decode;
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
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.acmerobotics.roadrunner.ParallelAction;;

@Config
@Autonomous
public class auto extends LinearOpMode {
    double jiaoduservoUp = 0;
    double jiaoduservoDown = 0.37;
    double jiaoduservoPing = 0.3;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);



        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
//        chelunServo.setPower(-1);

        Actions.runBlocking(new SequentialAction(drive.actionBuilder(beginPose)
//                .strafeTo(new Vector2d(0,0))
//                .splineToLinearHeading(new Pose2d(0,0,Math.PI),0)
//                .stopAndAdd(new action)

                        .build())


        );
    }

    public class xiSample implements Action {
        CRServo chelunServo = hardwareMap.get(CRServo.class, "servo5");
        DcMotor shengsuobi = hardwareMap.get(DcMotor.class, "motor4");
        Servo jiaoduServo = hardwareMap.get(Servo.class, "servo0");


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengsuobi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengsuobi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengsuobi.setDirection(DcMotorSimple.Direction.REVERSE);

            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;
//
                chelunServo.setPower(-1);
                shengsuobi.setTargetPosition(1200);
                shengsuobi.setPower(1);
                shengsuobi.setMode(RunMode.RUN_TO_POSITION);
                jiaoduServo.setPosition(jiaoduservoDown);

                if(currentTime > 2300){
                    jiaoduServo.setPosition(jiaoduservoPing);
                    return false;
                }

                telemetry.addData("shengjiangbi", shengsuobi.getCurrentPosition());
                telemetry.addData("chelunServo", chelunServo.getPower());
                telemetry.addData("jiaoduServo", jiaoduServo.getPosition());
                telemetry.update();

            }
        }

    }





//


    public class shouhuishengsuobi implements Action {

        DcMotor shengsuobi = hardwareMap.get(DcMotor.class, "motor4");


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;

                shengsuobi.setTargetPosition(0);
                shengsuobi.setPower(-1);
                shengsuobi.setMode(RunMode.RUN_TO_POSITION);

//                if(currentTime > 1000){
//                    return false;
//                }


                telemetry.addData("sengjiangbi", shengsuobi.getCurrentPosition());
                telemetry.update();
                return false;

            }
        }

    }




    public class tuSample implements Action {
        CRServo chelunServo = hardwareMap.get(CRServo.class, "servo5");
        DcMotor shengsuobi = hardwareMap.get(DcMotor.class, "motor4");
        Servo jiaoduServo = hardwareMap.get(Servo.class, "servo0");


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;
                jiaoduServo.setPosition(jiaoduservoPing);
                chelunServo.setPower(1);

                if(currentTime > 500){
                    chelunServo.setPower(0);
                    return false;
                }

//
                telemetry.addData("sengjiangbi", shengsuobi.getCurrentPosition());
                telemetry.addData("chelunServo", chelunServo.getPower());
                telemetry.addData("jiaoduServo", jiaoduServo.getPosition());
                telemetry.update();

            }
        }

    }

    public class shengguaSample implements Action {
        DcMotor shengjiangbi = hardwareMap.get(DcMotor.class, "motor5");

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengjiangbi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengjiangbi.setDirection(DcMotorSimple.Direction.REVERSE);


            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;

                shengjiangbi.setTargetPosition(2034);
                shengjiangbi.setPower(1);
                shengjiangbi.setMode(RunMode.RUN_TO_POSITION);


                telemetry.addData("shengjiangbi", shengjiangbi.getCurrentPosition());
                telemetry.update();
                return false;

            }
        }
    }

    public class naSample implements Action {
        DcMotor shengjiangbi = hardwareMap.get(DcMotor.class, "motor5");

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengjiangbi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengjiangbi.setDirection(DcMotorSimple.Direction.REVERSE);


            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;

                shengjiangbi.setTargetPosition(2034);
                shengjiangbi.setPower(1);
                shengjiangbi.setMode(RunMode.RUN_TO_POSITION);


                telemetry.addData("shengjiangbi", shengjiangbi.getCurrentPosition());
                telemetry.update();
                if (currentTime > 500) {
                    return false;
                }


            }
        }
    }

    public class xiangshangsongkaiSample implements Action {
        DcMotor shengjiangbi = hardwareMap.get(DcMotor.class, "motor5");

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengjiangbi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengjiangbi.setDirection(DcMotorSimple.Direction.REVERSE);


            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;

                shengjiangbi.setTargetPosition(2034-1000);
                shengjiangbi.setPower(1);
                shengjiangbi.setMode(RunMode.RUN_TO_POSITION);


                telemetry.addData("shengjiangbi", shengjiangbi.getCurrentPosition());
                telemetry.update();
                if (currentTime > 1000) {
                    return false;
                }


            }
        }
    }

    public class jiangguaSample implements Action {
        DcMotor shengjiangbi = hardwareMap.get(DcMotor.class, "motor5");

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengjiangbi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengjiangbi.setDirection(DcMotorSimple.Direction.REVERSE);

            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;



                shengjiangbi.setTargetPosition(900 - 2034);
                shengjiangbi.setPower(-1);
                shengjiangbi.setMode(RunMode.RUN_TO_POSITION);

                telemetry.addData("shengjiangbi", shengjiangbi.getCurrentPosition());
                telemetry.update();


                if (currentTime > 1000) {
                    return false;
                }

            }
        }
    }

    public class bifangdaozuixia implements Action {
        DcMotor shengjiangbi = hardwareMap.get(DcMotor.class, "motor5");

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengjiangbi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengjiangbi.setDirection(DcMotorSimple.Direction.REVERSE);

            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;

                shengjiangbi.setTargetPosition(0 - 900);
                shengjiangbi.setPower(-1);
                shengjiangbi.setMode(RunMode.RUN_TO_POSITION);


                telemetry.addData("shengjiangbi", shengjiangbi.getCurrentPosition());
                telemetry.update();

                return false;
//


            }
        }


    }
}