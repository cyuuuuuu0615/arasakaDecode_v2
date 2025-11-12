package org.firstinspires.ftc.teamcode.decode.AprilTag;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Red Team Tracker", group = "Competition")
public class RedTeamTracker extends LinearOpMode {

    private Limelight3A limelight;
    private Servo panServo;

    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_CENTER = 0.5;

    // 穩定化參數
    private static final double MOVE_FACTOR = 0.0008;
    private static final double DEAD_ZONE = 0.8;
    private static final double MAX_MOVE_PER_FRAME = 0.002;
    private static final double MIN_MOVE_THRESHOLD = 0.0003;

    // 穩定狀態追蹤
    private double lastTx = 0;
    private int stableCount = 0;
    private static final int STABLE_THRESHOLD = 5;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        panServo = hardwareMap.get(Servo.class, "servo0");

        // 設定為紅隊管道 8 (檢測 20 號 AprilTag)
        limelight.pipelineSwitch(8);
        panServo.setPosition(SERVO_CENTER);

        telemetry.addData("隊伍", "🔴 紅隊");
        telemetry.addData("目標 AprilTag", "20 號");
        telemetry.addData("管道", "8 - redAprilTag");
        telemetry.addData("狀態", "穩定直接追蹤 - 準備就緒");
        telemetry.update();

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            trackStable();
            telemetry.update();
            sleep(10);
        }

        limelight.stop();
    }

    private void trackStable() {
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double ta = llResult.getTa();

            // 檢測目標是否穩定
            boolean isStable = Math.abs(tx - lastTx) < 0.3;
            if (isStable) {
                stableCount++;
            } else {
                stableCount = 0;
            }

            telemetry.addData("隊伍", "🔴 紅隊 - 追蹤 20 號");
            telemetry.addData("tx", "%.2f°", tx);
            telemetry.addData("tx變化", "%.2f", tx - lastTx);
            telemetry.addData("穩定計數", stableCount);

            if (Math.abs(tx) > DEAD_ZONE) {
                double moveAmount = tx * MOVE_FACTOR;

                // 當目標穩定時，減少移動量
                if (stableCount > STABLE_THRESHOLD) {
                    moveAmount *= 0.3;
                    telemetry.addData("移動模式", "穩定模式");
                } else {
                    telemetry.addData("移動模式", "追蹤模式");
                }

                // 限制最大移動量
                if (moveAmount > MAX_MOVE_PER_FRAME) {
                    moveAmount = MAX_MOVE_PER_FRAME;
                } else if (moveAmount < -MAX_MOVE_PER_FRAME) {
                    moveAmount = -MAX_MOVE_PER_FRAME;
                }

                // 忽略過小的移動（減少微震盪）
                if (Math.abs(moveAmount) < MIN_MOVE_THRESHOLD) {
                    telemetry.addData("動作", "跳過微調");
                } else {
                    double newPosition = panServo.getPosition() + moveAmount;
                    newPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, newPosition));

                    panServo.setPosition(newPosition);

                    telemetry.addData("動作", "移動 → %.4f", newPosition);
                    telemetry.addData("移動量", "%.4f", moveAmount);
                }
            } else {
                telemetry.addData("動作", "目標置中");
                stableCount = 0;
            }

            lastTx = tx;

        } else {
            telemetry.addData("隊伍", "🔴 紅隊");
            telemetry.addData("狀態", "❌ 未找到 20 號 AprilTag");
            stableCount = 0;
        }

        telemetry.addData("當前位置", "%.4f", panServo.getPosition());
    }
}