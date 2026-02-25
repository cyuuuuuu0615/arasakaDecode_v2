package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Test Turret Direction (Safety)")
public class TestTurretDirection extends LinearOpMode {

    private DcMotorEx turretMotor;

    @Override
    public void runOpMode() {
        // 1. 硬體映射 (請確認你的 Config 裡面馬達名字是不是 "turret" 或 "slide")
        // 如果你的 Config 名字不同，請修改下面這一行綠色的字
        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "motor6");
        } catch (Exception e) {
            telemetry.addData("錯誤", "找不到馬達！請檢查 Config 名字是否為 'motor6'");
            telemetry.update();
            waitForStart();
            return;
        }

        // 2. 初始化設定
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 煞車模式，防止亂飄

        telemetry.addLine("準備完成！");
        telemetry.addLine("按 Start 後，請「輕輕」按 Right Trigger");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 3. 控制邏輯：使用 Trigger (板機) 來控制
            // Right Trigger = 給正電 (Positive Power)
            // Left Trigger  = 給負電 (Negative Power)

            double power = gamepad1.right_trigger - gamepad1.left_trigger;

            // 安全限制：只給 30% 力量，避免轉太快扯斷線
            power = power * 0.3;

            turretMotor.setPower(power);

            // 4. 數據回傳
            telemetry.addData("馬達力量 (Power)", "%.2f", power);
            telemetry.addData("編碼器位置 (Position)", turretMotor.getCurrentPosition());
            telemetry.addLine("\n-----------------");

            if (power > 0) {
                telemetry.addData("狀態", "正向供電中 (Positive) >>>");
                telemetry.addData("請觀察", "砲塔是 順時針(CW) 還是 逆時針(CCW) 轉？");
            } else if (power < 0) {
                telemetry.addData("狀態", "<<< 負向供電中 (Negative)");
            } else {
                telemetry.addData("狀態", "停止 (Stop)");
            }

            telemetry.update();
        }
    }
}