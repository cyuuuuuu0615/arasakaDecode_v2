package org.firstinspires.ftc.teamcode.decode.AprilTag;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Fast PID Tracker", group = "Competition")
public class PIDservoTracker extends LinearOpMode {

    private Limelight3A limelight;
    private Servo panServo;

    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_CENTER = 0.5;

    // 快速響應 PID 參數
    private static final double KP = 0.001;
    private static final double KI = 0.000005;
    private static final double KD = 0.00008;

    private static final double DEAD_ZONE = 0.2;
    private static final double MAX_OUTPUT = 0.003;

    private double integral = 0;
    private double lastError = 0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        panServo = hardwareMap.get(Servo.class, "servo0");

        limelight.pipelineSwitch(0);
        panServo.setPosition(SERVO_CENTER);

        telemetry.addData("狀態", "快速 PID 追蹤 - 準備就緒");
        telemetry.update();

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();
                double error = tx; // 直接使用 tx 作為誤差

                // 快速 PID 計算
                double proportional = KP * error;

                // 有限的積分
                integral += error * 0.01; // 固定時間步長
                integral = Math.max(-30, Math.min(30, integral));
                double integralTerm = KI * integral;

                // 微分
                double derivative = KD * (error - lastError) / 0.01;

                double output = proportional + integralTerm + derivative;
                output = Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, output));

                double newPosition = panServo.getPosition() + output;
                newPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, newPosition));

                panServo.setPosition(newPosition);

                telemetry.addData("狀態", "🚀 快速追蹤");
                telemetry.addData("tx", "%.2f°", tx);
                telemetry.addData("位置", "%.4f", newPosition);
                telemetry.addData("輸出", "%.4f", output);

                lastError = error;
            } else {
                telemetry.addData("狀態", "❌ 無目標");
                integral = 0;
            }

            telemetry.update();
            sleep(8); // 超快循環
        }

        limelight.stop();
    }
}

//package org.firstinspires.ftc.teamcode.decode.AprilTag;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@TeleOp(name = "PD Tracker", group = "Competition")
//public class PIDservoTracker extends LinearOpMode {
//
//    private Limelight3A limelight;
//    private Servo panServo;
//
//    private static final double SERVO_MIN = 0.0;
//    private static final double SERVO_MAX = 1.0;
//    private static final double SERVO_CENTER = 0.5;
//
//    // 只使用 PD 控制（移除積分項）
//    private static final double KP = 0.0012;
//    private static final double KD = 0.0002;
//
//    private static final double DEAD_ZONE = 0.25;
//    private static final double MAX_OUTPUT = 0.0028;
//
//    private double lastError = 0;
//    private long lastTime = 0;
//
//    @Override
//    public void runOpMode() {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        panServo = hardwareMap.get(Servo.class, "servo0");
//
//        limelight.pipelineSwitch(0);
//        panServo.setPosition(SERVO_CENTER);
//
//        telemetry.addData("狀態", "PD 控制追蹤 - 無震盪");
//        telemetry.addData("特點", "移除積分項，避免搖擺");
//        telemetry.update();
//
//        waitForStart();
//
//        limelight.start();
//
//        while (opModeIsActive()) {
//            trackWithPD();
//            telemetry.update();
//            sleep(10);
//        }
//
//        limelight.stop();
//    }
//
//    private void trackWithPD() {
//        LLResult llResult = limelight.getLatestResult();
//
//        if (llResult != null && llResult.isValid()) {
//            double tx = llResult.getTx();
//            double error = tx;
//
//            long currentTime = System.currentTimeMillis();
//            double dt = Math.max(0.001, (currentTime - lastTime) / 1000.0);
//
//            telemetry.addData("狀態", "🎯 PD 追蹤");
//            telemetry.addData("tx", "%.2f°", tx);
//
//            if (Math.abs(error) > DEAD_ZONE) {
//                // PD 控制計算
//                double proportional = KP * error;
//                double derivative = KD * (error - lastError) / dt;
//
//                double output = proportional + derivative;
//                output = Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, output));
//
//                double newPosition = panServo.getPosition() + output;
//                newPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, newPosition));
//
//                panServo.setPosition(newPosition);
//
//                telemetry.addData("動作", "移動 → %.4f", newPosition);
//                telemetry.addData("P 輸出", "%.4f", proportional);
//                telemetry.addData("D 輸出", "%.4f", derivative);
//            } else {
//                telemetry.addData("動作", "完美置中");
//            }
//
//            lastTime = currentTime;
//            lastError = error;
//
//        } else {
//            telemetry.addData("狀態", "❌ 無目標");
//            lastError = 0;
//        }
//
//        telemetry.addData("位置", "%.4f", panServo.getPosition());
//    }
//}

