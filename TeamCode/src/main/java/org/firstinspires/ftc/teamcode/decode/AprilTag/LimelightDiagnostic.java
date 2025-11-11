package org.firstinspires.ftc.teamcode.decode.AprilTag;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Limelight Diagnostic", group = "Diagnostic")
public class LimelightDiagnostic extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Limelight 診斷工具");
        telemetry.addData("裝置名稱", "limelight");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 使用正確的裝置名稱 "limelight"
            double tv = getLimelightValue("tv");
            double tx = getLimelightValue("tx");
            double ty = getLimelightValue("ty");
            double ta = getLimelightValue("ta");
            double tid = getLimelightValue("tid");
            double pipeline = getLimelightValue("pipeline");

            telemetry.addData("=== Limelight 診斷 ===", "");
            telemetry.addData("裝置名稱", "limelight");
            telemetry.addData("是否有目標 (tv)", "%.0f", tv);
            telemetry.addData("AprilTag ID (tid)", "%.0f", tid);
            telemetry.addData("水平偏移 (tx)", "%.2f°", tx);
            telemetry.addData("垂直偏移 (ty)", "%.2f°", ty);
            telemetry.addData("目標面積 (ta)", "%.2f%%", ta * 100); // 轉換為百分比
            telemetry.addData("當前管道", "%.0f", pipeline);
            telemetry.addData("", "");

            if (tv == 0) {
                telemetry.addData("狀態", "❌ 未檢測到 AprilTag");
                telemetry.addData("解決步驟", "1. 檢查 AprilTag 是否在視野內");
                telemetry.addData("", "2. 確認距離 1-3 米");
                telemetry.addData("", "3. 檢查光線充足");
                telemetry.addData("", "4. 確認管道設定正確");
            } else {
                telemetry.addData("狀態", "✅ 檢測到 AprilTag #%.0f", tid);
                telemetry.addData("建議", "可以開始追蹤!");
            }

            telemetry.update();
            sleep(100);
        }
    }

    private double getLimelightValue(String key) {
        try {
            // 使用正確的裝置名稱 "limelight"
            String value = hardwareMap.appContext.getSharedPreferences("limelight", 0)
                    .getString(key, "0");
            return Double.parseDouble(value);
        } catch (Exception e) {
            telemetry.addData("錯誤", "讀取 Limelight 數據失敗: " + key);
            return 0.0;
        }
    }
}