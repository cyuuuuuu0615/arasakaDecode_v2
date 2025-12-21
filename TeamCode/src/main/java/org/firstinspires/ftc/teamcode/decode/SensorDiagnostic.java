package org.firstinspires.ftc.teamcode.decode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@TeleOp(name = "Sensor_Diagnostic_Test", group = "Test")
public class SensorDiagnostic extends LinearOpMode {

    // 定義感測器變數
    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {
        telemetry.addData("狀態", "初始化硬體中...");
        telemetry.update();

        try {
            // 1. 獲取硬體
            // 請務必確認 Robot Config 中的名字是 "colorSensor0"
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");

            // 2. 嘗試開啟補光燈 (如果硬體支援)
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(true);
            }

            // 3. 設定高增益 (Gain) 以確保能讀到微弱光線
            colorSensor.setGain(15);

            telemetry.addData("狀態", "硬體獲取成功");
            telemetry.addData("連接資訊", colorSensor.getConnectionInfo());

        } catch (Exception e) {
            telemetry.addData("錯誤", "無法獲取感測器！");
            telemetry.addData("原因", "請檢查 Config 名稱是否為 colorSensor0");
            telemetry.update();
            // 如果抓不到硬體，程式會在這裡卡住等待停止
            waitForStart();

        }

        telemetry.addData("操作說明", "按 A 鍵切換開關燈");
        telemetry.update();

        waitForStart();

        boolean lightEnabled = true;
        boolean lastA = false;

        while (opModeIsActive()) {
            // --- 功能 1: 按 A 鍵測試燈光開關 (測試 I2C 寫入功能) ---
            if (gamepad1.a && !lastA) {
                lightEnabled = !lightEnabled;
                if (colorSensor instanceof SwitchableLight) {
                    ((SwitchableLight) colorSensor).enableLight(lightEnabled);
                }
            }
            lastA = gamepad1.a;

            // --- 功能 2: 讀取數據 (測試 I2C 讀取功能) ---
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // 轉換為 HSV 方便觀察
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);

            // --- 顯示詳細診斷資訊 ---
            telemetry.addLine("=== 硬體診斷 ===");
            telemetry.addData("LED 狀態", lightEnabled ? "開啟" : "關閉");
            telemetry.addData("連接資訊", colorSensor.getConnectionInfo());
            telemetry.addData("Gain (增益)", colorSensor.getGain());

            telemetry.addLine("\n=== 原始數據 (Raw) ===");
            // 正常情況下，這些數值不應該是全 0
            telemetry.addData("Red", "%.4f", colors.red);
            telemetry.addData("Green", "%.4f", colors.green);
            telemetry.addData("Blue", "%.4f", colors.blue);
            telemetry.addData("Alpha (亮度)", "%.4f", colors.alpha);

            telemetry.addLine("\n=== 色彩分析 ===");
            telemetry.addData("Hue (色相)", "%.1f", hsvValues[0]);
            telemetry.addData("Sat (飽和)", "%.2f", hsvValues[1]);
            telemetry.addData("Val (亮度)", "%.2f", hsvValues[2]);

            // --- 故障判斷邏輯 ---
            telemetry.addLine("\n=== 診斷結論 ===");
            if (colors.alpha == 0.0 && colors.red == 0.0 && colors.green == 0.0 && colors.blue == 0.0) {
                telemetry.addData("狀態", "❌ 通訊失敗 (全為 0)");
                telemetry.addData("建議", "線材斷了 或 插錯孔 (I2C vs Analog)");
            } else if (colors.alpha < 0.005) {
                telemetry.addData("狀態", "⚠️ 數值極低 (接近全黑)");
                telemetry.addData("建議", "請檢查燈是否亮著，或將物體靠近");
            } else {
                telemetry.addData("狀態", "✅ 感測器運作正常");
            }

            telemetry.update();
        }

    }
}