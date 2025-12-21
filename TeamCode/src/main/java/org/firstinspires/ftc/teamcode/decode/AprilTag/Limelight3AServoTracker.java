package org.firstinspires.ftc.teamcode.decode.AprilTag;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Limelight3A Servo Tracker_v1", group = "Competition")
public class Limelight3AServoTracker extends LinearOpMode {

    private Limelight3A limelight;
    private Servo panServo;

    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_CENTER = 0.5;

    // 超穩定參數
    private static final double MOVE_FACTOR = 0.0006;
    private static final double DEAD_ZONE = 1.0; // 更大的死區
    private static final double MIN_MOVE = 0.0005; // 忽略更小的移動

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        panServo = hardwareMap.get(Servo.class, "servo0");

        limelight.pipelineSwitch(0);
        panServo.setPosition(SERVO_CENTER);

        telemetry.addData("狀態", "超穩定追蹤 - 準備就緒");
        telemetry.update();

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();

                // 簡單判斷：只有當偏移足夠大時才移動
                if (Math.abs(tx) > DEAD_ZONE) {
                    double moveAmount = tx * MOVE_FACTOR;

                    // 忽略過小的移動
                    if (Math.abs(moveAmount) > MIN_MOVE) {
                        double newPosition = panServo.getPosition() + moveAmount;
                        newPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, newPosition));

                        panServo.setPosition(newPosition);

                        telemetry.addData("狀態", "✅ 穩定移動");
                        telemetry.addData("tx", "%.2f°", tx);
                        telemetry.addData("位置", "%.4f", newPosition);
                    } else {
                        telemetry.addData("狀態", "✅ 置中（忽略微調）");
                        telemetry.addData("tx", "%.2f°", tx);
                    }
                } else {
                    telemetry.addData("狀態", "✅ 完美置中");
                    telemetry.addData("tx", "%.2f°", tx);
                }
            } else {
                telemetry.addData("狀態", "❌ 無目標");
            }

            telemetry.update();
            sleep(12);
        }

        limelight.stop();

    }
}