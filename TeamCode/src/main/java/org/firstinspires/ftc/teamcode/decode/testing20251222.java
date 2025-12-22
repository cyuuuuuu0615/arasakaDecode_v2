package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "testing20251222")
public class testing20251222 extends LinearOpMode {

    // 定義硬體變數
    private DcMotor motor5;

    @Override
    public void runOpMode() {
        // --- Initialization (初始化) ---

        // 獲取馬達硬體
        motor5 = hardwareMap.get(DcMotor.class, "motor5");

        // 定義馬達功率變數
        double motorPowerVariable = 0;

        // --- 計時器變數設定 ---
        // 用來記錄上一次按下按鈕的時間
        long lastInputTime = 0;
        // 設定冷卻時間 (毫秒)，這裡設為 200ms (0.2秒)
        long inputDelay = 200;

        waitForStart();

        // --- Run Loop (主程式) ---
        if (opModeIsActive()) {

            // 重置功率
            motorPowerVariable = 0;

            while (opModeIsActive()) {

                // 獲取當前系統時間
                long currentTime = System.currentTimeMillis();

                // 1. 檢測手把輸入 (加入時間判斷)

                // 邏輯：如果 (按下了左鍵) 且 (現在時間 - 上次按下的時間 > 延遲時間)
                if (gamepad1.dpad_left && (currentTime - lastInputTime > inputDelay)) {

                    motorPowerVariable = motorPowerVariable + 0.1;

                    // 更新 "上次按下時間" 為 "現在時間"
                    lastInputTime = currentTime;
                }
                // 邏輯：如果 (按下了右鍵) 且 (冷卻時間已到)
                else if (gamepad1.dpad_right && (currentTime - lastInputTime > inputDelay)) {

                    motorPowerVariable = motorPowerVariable - 0.1;

                    // 更新計時器
                    lastInputTime = currentTime;
                }

                // --- 安全限制 (可選) ---
                // 為了防止數值超過 1.0 或 低於 -1.0，建議加上這段 (Block 裡沒有，但 Java 建議加)
                if (motorPowerVariable > 1.0) motorPowerVariable = 1.0;
                if (motorPowerVariable < -1.0) motorPowerVariable = -1.0;

                // 2. 設定馬達功率
                motor5.setPower(motorPowerVariable);

                // 3. 數據回傳 (Telemetry)
                telemetry.addData("Status", "Running");
                telemetry.addData("Target Power", motorPowerVariable);
                telemetry.addData("Actual Motor Power", motor5.getPower());
                telemetry.update();
            }
        }
    }
}