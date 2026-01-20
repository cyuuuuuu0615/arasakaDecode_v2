package org.firstinspires.ftc.teamcode.decode.AprilTag;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@TeleOp(name = "Motor_v1")
public class Motor_v1 extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor turretMotor;

    private final double TARGET_TX = 8.0;
    private final double KP = 0.02;
    private final double MAX_POWER = 0.3;
    private final double MIN_POWER = 0.08;
    private final double DEADBAND = 1.0;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotor.class, "motor6");
        turretMotor.setDirection(Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(0);
        limelight.start();

        // 初始化期間的 Telemetry
        telemetry.addData("Status", "Initialized - Waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double error = tx - TARGET_TX;
                double power = 0;

                if (Math.abs(error) > DEADBAND) {
                    power = error * KP;

                    if (error > 0) {
                        power = power + MIN_POWER;
                    } else {
                        power = power - MIN_POWER;
                    }

                    power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
                    turretMotor.setPower(power);
                } else {
                    turretMotor.setPower(0);
                }

                // 顯示追蹤中的數據
                telemetry.addData("Target Status", "LOCKED");
                telemetry.addData("Limelight tx", "%.2f", tx);
                telemetry.addData("Error", "%.2f", error);
                telemetry.addData("Motor Power", "%.3f", power);
            } else {
                turretMotor.setPower(0);
                telemetry.addData("Target Status", "LOST - Searching...");
            }

            // 無論是否抓到目標都顯示的數據
            telemetry.addData("Encoder Position", turretMotor.getCurrentPosition());
            telemetry.addLine("---------------------------");
            telemetry.addData("Configuration", "Target:%s, MinPow:%s", TARGET_TX, MIN_POWER);
            telemetry.update(); // 務必呼叫 update() 才會刷新螢幕
        }
        limelight.stop();
    }
}