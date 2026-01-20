package org.firstinspires.ftc.teamcode.decode.AprilTag;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@TeleOp(name = "A Motor_v3")
public class Motor_v3 extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor turretMotor;

    // --- 關鍵參數調整 (建議先從低數值開始) ---
    private final double TARGET_TX = 8.0;

    // P 決定追蹤力量：如果還是衝太快，請將 0.03 降到 0.02
    private double KP = 0.018;

    // D 決定煞車力量：如果衝過頭，請增加這個值 (嘗試 0.01 -> 0.015 -> 0.02)
    private double KD = 0.025;

    // 限制物理速度：先降到 0.4 確保它不會飛出去，穩定後再調高
    private final double MAX_POWER = 0.45;

    // 最小啟動動力：降到剛好能動即可，避免「彈射」起步
    private final double MIN_POWER = 0.06;

    private final double DEADBAND = 1.0;
    private double lastError = 0;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotor.class, "motor6");
        turretMotor.setDirection(Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 確保停止時馬達鎖死

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double error = tx - TARGET_TX;

                // 計算 D 項：誤差的變化率 (error - lastError)
                double errorChange = error - lastError;
                double dTerm = errorChange * KD;

                if (Math.abs(error) > DEADBAND) {
                    // PD 控制公式
                    double power = (error * KP) + dTerm;

                    // 加上最小動力補償
                    if (error > 0) power += MIN_POWER;
                    else power -= MIN_POWER;

                    // 最終限制
                    power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
                    turretMotor.setPower(power);
                } else {
                    turretMotor.setPower(0);
                }

                lastError = error; // 更新上一次誤差

                telemetry.addData("Status", "Tracking");
                telemetry.addData("tx", "%.2f", tx);
            } else {
                // ！！！重要：失去目標時不要立即讓動力變 0，可以緩慢減速
                // 這裡先設為 0 以保安全，但如果想更強，可以加入慣性維持
                turretMotor.setPower(0);
                lastError = 0;
                telemetry.addData("Status", "LOST - Searching");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}