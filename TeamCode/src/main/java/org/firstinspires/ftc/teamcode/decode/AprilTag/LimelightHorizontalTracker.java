//package org.firstinspires.ftc.teamcode.decode.AprilTag;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@TeleOp(name = "Limelight Horizontal Tracker", group = "Competition")
//public class LimelightHorizontalTracker extends LinearOpMode {
//
//    private Servo panServo;    // 水平伺服馬達
//
//    // 伺服馬達角度範圍
//    private static final double SERVO_MIN = 0.0;    // 最左邊
//    private static final double SERVO_MAX = 1.0;    // 最右邊
//    private static final double SERVO_CENTER = 0.5; // 中心位置
//
//    // 追蹤參數
//    private static final double MOVE_SPEED = 0.015; // 移動速度
//    private static final double DEAD_ZONE = 2.0;    // 死區範圍 (度)
//
//    // Limelight 網路表格鍵值
//    private static final String LIMELIGHT_TX = "tx"; // 水平偏移角度 (度)
//    private static final String LIMELIGHT_TV = "tv"; // 是否有目標 (0或1)
//
//    @Override
//    public void runOpMode() {
//        initializeHardware();
//
//        telemetry.addData("Status", "初始化完成");
//        telemetry.addData("說明", "Limelight 水平追蹤 AprilTag");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            trackHorizontal();
//            telemetry.update();
//            sleep(20);
//        }
//    }
//
//    private void initializeHardware() {
//        // 初始化伺服馬達
//        panServo = hardwareMap.get(Servo.class, "servo0");
//
//        // 重置到中心位置
//        panServo.setPosition(SERVO_CENTER);
//
//        telemetry.addData("Servo Position", "中心: %.3f", SERVO_CENTER);
//    }
//
//    private void trackHorizontal() {
//        // 檢查是否有檢測到目標
//        boolean hasTarget = getLimelightDouble(LIMELIGHT_TV) == 1.0;
//
//        if (hasTarget) {
//            // 獲取水平偏移角度 (單位: 度)
//            double tx = getLimelightDouble(LIMELIGHT_TX);
//
//            // 如果偏移在死區範圍內，保持不動
//            if (Math.abs(tx) > DEAD_ZONE) {
//                // 計算新的伺服馬達位置
//                // tx > 0: 目標在右邊，伺服馬達需要向右轉
//                // tx < 0: 目標在左邊，伺服馬達需要向左轉
//                double moveAmount = -tx * MOVE_SPEED; // 負號是因為座標系相反
//                double newPosition = panServo.getPosition() + moveAmount;
//
//                // 限制在有效範圍內
//                newPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, newPosition));
//
//                // 設定新位置
//                panServo.setPosition(newPosition);
//
//                telemetry.addData("Status", "追蹤中...");
//                telemetry.addData("動作", "移動伺服馬達");
//            } else {
//                telemetry.addData("Status", "目標已置中");
//                telemetry.addData("動作", "保持位置");
//            }
//
//            // 顯示追蹤資訊
//            telemetry.addData("目標偏移 tx", "%.2f°", tx);
//            telemetry.addData("死區範圍", "±%.1f°", DEAD_ZONE);
//            telemetry.addData("伺服馬達位置", "%.3f", panServo.getPosition());
//
//        } else {
//            telemetry.addData("Status", "未檢測到目標");
//            telemetry.addData("伺服馬達位置", "%.3f", panServo.getPosition());
//            telemetry.addData("建議", "檢查 Limelight 畫面是否有 AprilTag");
//        }
//    }
//
//    /**
//     * 從 Limelight 網路表格獲取數值
//     */
//    private double getLimelightDouble(String key) {
//        try {
//            return Double.parseDouble(hardwareMap.appContext.getSharedPreferences("Limelight", 0).getString(key, "0"));
//        } catch (Exception e) {
//            return 0.0;
//        }
//    }
//
//    /**
//     * 設定 Limelight 管道 (可選)
//     */
//    private void setLimelightPipeline(int pipeline) {
//        try {
//            hardwareMap.appContext.getSharedPreferences("Limelight", 0)
//                    .edit()
//                    .putString("pipeline", String.valueOf(pipeline))
//                    .apply();
//        } catch (Exception e) {
//            telemetry.addData("錯誤", "無法設定 Limelight 管道");
//        }
//    }
//}