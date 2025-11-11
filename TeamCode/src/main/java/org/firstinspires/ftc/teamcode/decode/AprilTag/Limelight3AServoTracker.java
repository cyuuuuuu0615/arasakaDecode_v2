package org.firstinspires.ftc.teamcode.decode.AprilTag;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Limelight3A Servo Tracker", group = "Competition")
public class Limelight3AServoTracker extends LinearOpMode {

    private Limelight3A limelight;
    private Servo panServo;
    private IMU imu;

    // 伺服馬達參數
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_CENTER = 0.5;

    // 追蹤參數
    private static final double MOVE_SPEED = 0.01;
    private static final double DEAD_ZONE = 2.0; // 度

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("狀態", "初始化完成 - 等待開始");
        telemetry.addData("Limelight", "管道 0 - AprilTag 檢測");
        telemetry.addData("伺服馬達", "中心位置: %.3f", SERVO_CENTER);
        telemetry.update();

        waitForStart();

        // 啟動 Limelight
        limelight.start();

        while (opModeIsActive()) {
            trackAprilTag();
            telemetry.update();
            sleep(50);
        }

        // 停止時關閉 Limelight
        limelight.stop();
    }

    private void initializeHardware() {
        // 初始化 Limelight3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // 設定為 AprilTag 管道

        // 初始化伺服馬達
        panServo = hardwareMap.get(Servo.class, "servo0");
        panServo.setPosition(SERVO_CENTER);

        // 初始化 IMU (可選，用於機器人方向)
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));
    }

    private void trackAprilTag() {
        // 獲取 Limelight 最新結果
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // 獲取目標數據
            double tx = llResult.getTx(); // 水平偏移角度 (度)
            double ty = llResult.getTy(); // 垂直偏移角度 (度)
            double ta = llResult.getTa(); // 目標面積

            // 顯示檢測資訊
            telemetry.addData("狀態", "✅ 檢測到 AprilTag");
            telemetry.addData("水平偏移 tx", "%.2f°", tx);
            telemetry.addData("垂直偏移 ty", "%.2f°", ty);
            telemetry.addData("目標面積 ta", "%.3f", ta);

            // 追蹤邏輯
            if (Math.abs(tx) > DEAD_ZONE) {
                // 計算移動量
                // tx > 0: 目標在右邊，伺服馬達需要向右轉
                // tx < 0: 目標在左邊，伺服馬達需要向左轉
                double moveAmount = -tx * MOVE_SPEED;
                double newPosition = panServo.getPosition() + moveAmount;

                // 限制在有效範圍內
                newPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, newPosition));

                // 設定新位置
                panServo.setPosition(newPosition);

                telemetry.addData("動作", "追蹤中 → 新位置: %.3f", newPosition);
                telemetry.addData("移動量", "%.4f", moveAmount);
            } else {
                telemetry.addData("動作", "目標已置中 - 保持位置");
            }

            telemetry.addData("當前伺服位置", "%.3f", panServo.getPosition());
            telemetry.addData("死區範圍", "±%.1f°", DEAD_ZONE);

        } else {
            // 未檢測到目標
            telemetry.addData("狀態", "❌ 未檢測到 AprilTag");
            telemetry.addData("當前伺服位置", "%.3f (保持)", panServo.getPosition());
            telemetry.addData("建議", "請確保:");
            telemetry.addData("- AprilTag 在視野內", "");
            telemetry.addData("- 距離 0.5-3 米", "");
            telemetry.addData("- 光線充足", "");

            // 手動控制備用
            manualControl();
        }
    }

    private void manualControl() {
        // 手動控制備用方案
        boolean manualMoved = false;

        if (gamepad1.dpad_left) {
            panServo.setPosition(panServo.getPosition() - 0.01);
            manualMoved = true;
        }
        if (gamepad1.dpad_right) {
            panServo.setPosition(panServo.getPosition() + 0.01);
            manualMoved = true;
        }
        if (gamepad1.a) {
            panServo.setPosition(SERVO_CENTER);
            manualMoved = true;
            telemetry.addData("手動控制", "已重置到中心位置");
        }

        if (manualMoved) {
            telemetry.addData("手動控制", "←左轉 →右轉 A:中心");
        }
    }
}