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
public class testAuto extends LinearOpMode {
    double jiaoduservoUp = 0;
    double jiaoduservoDown = 0.37;
    double jiaoduservoPing = 0.3;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);



        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(new SequentialAction(drive.actionBuilder(beginPose)
                // 24.7 per one, x forward positive
                // 33 per one, y left positive
                        .strafeTo(new Vector2d(86.1, 33))
                // add shooting action
//                .splineToLinearHeading(new Pose2d(168,58,0),Math.PI)
//                .strafeTo(new Vector2d(168,-100))


//                        .stopAndAdd(new jiangguaSample())
//// **               .strafeTo(new Vector2d(-27,0))
//                        .strafeTo(new Vector2d(-20, 0))
//
//                        .stopAndAdd(new bifangdaozuixia())
//                        .splineToLinearHeading(new Pose2d(-5,20,Math.PI),0)
//                        .strafeTo(new Vector2d(1.5,20))
//                        .stopAndAdd(new naSample())
//                        .splineToLinearHeading(new Pose2d(-30,5,0),Math.PI)
//                        .stopAndAdd(new jiangguaSample())
//// **                       .strafeTo(new Vector2d(-27,5))
//                        .strafeTo(new Vector2d(-20,5))
//
//                        .stopAndAdd(new bifangdaozuixia())
//
//
//
////-------------------------------------------------------------------
//                        .splineToLinearHeading(new Pose2d(-25, 18, Math.PI*0.66), 0)
//                        .stopAndAdd(new xiSample())
//                        .splineToLinearHeading(new Pose2d(-28,25,Math.PI*0.2),Math.PI*0.66)
////                .turnTo(Math.PI*0.2)
//                        .stopAndAdd(new tuSample())
//                        //18s
//                        //-----------------------------------------------------------
//                        .stopAndAdd(new shouhuishengsuobi())
//                        .splineToLinearHeading(new Pose2d(-5,20,Math.PI),Math.PI*0.3)
//                        .waitSeconds(2)
//                        .strafeTo(new Vector2d(1.5,20))
//                        .stopAndAdd(new naSample())
//                        .splineToLinearHeading(new Pose2d(-20,10,0),Math.PI)
//                        .strafeTo(new Vector2d(-30,10))
//                        .stopAndAdd(new jiangguaSample())
////        **                .strafeTo(new Vector2d(-28,10))
//                        .splineToLinearHeading(new Pose2d(-20,10,Math.PI),0)
//
//
//                        .stopAndAdd(new bifangdaozuixia())
//                        //
//
//
////
//                        .strafeTo(new Vector2d(0,30))
                        .build())

////                .strafeTo(new Vector2d(0,0))
////                .splineToLinearHeading(new Pose2d(0,0,Math.PI),0)
////                .stopAndAdd(new action)
//
//                        .build())
//

        );

    }

    

    public class xiSample implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            long startTime = System.currentTimeMillis();
            while (true) {

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