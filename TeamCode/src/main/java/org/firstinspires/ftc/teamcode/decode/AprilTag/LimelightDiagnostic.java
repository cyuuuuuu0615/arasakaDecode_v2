//package org.firstinspires.ftc.teamcode.decode.AprilTag;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name = "Limelight Diagnostic", group = "Diagnostic")
//public class LimelightDiagnostic extends LinearOpMode {
//
//    private Limelight3A limelight;
//
//    @Override
//    public void runOpMode() {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        telemetry.addData("狀態", "Limelight 診斷工具");
//        telemetry.addData("說明", "檢查所有管道和設定");
//        telemetry.update();
//
//        waitForStart();
//
//        // 測試所有管道
//        for (int pipeline = 0; pipeline <= 9; pipeline++) {
//            testPipeline(pipeline);
//        }
//
//        telemetry.addData("診斷完成", "請查看哪個管道有數據");
//        telemetry.update();
//
//        while (opModeIsActive()) {
//            sleep(1000);
//        }
//    }
//
//    private void testPipeline(int pipeline) {
//        // 切換管道
//        limelight.pipelineSwitch(pipeline);
//        sleep(500); // 等待管道切換
//
//        // 啟動 Limelight
//        limelight.start();
//        sleep(1000); // 收集數據
//
//        // 獲取結果
//        LLResult llResult = limelight.getLatestResult();
//
//        telemetry.addData("=== 管道 " + pipeline + " 測試 ===", "");
//        telemetry.addData("llResult", llResult != null ? "非空" : "NULL");
//
//        if (llResult != null) {
//            telemetry.addData("isValid()", llResult.isValid());
//            telemetry.addData("getTx()", "%.2f", llResult.getTx());
//            telemetry.addData("getTy()", "%.2f", llResult.getTy());
//            telemetry.addData("getTa()", "%.3f", llResult.getTa());
//        } else {
//            telemetry.addData("錯誤", "llResult 為 NULL");
//        }
//
//        telemetry.update();
//        sleep(2000); // 暫停以便閱讀
//
//        limelight.stop();
//    }
//}