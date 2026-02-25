package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "teleop_v3_CALIBRATION_TOOL")
public class teleop_SpinPostionTest extends LinearOpMode {

    private CRServo diskServo;
    private DcMotor intakeMotor;

    @Override
    public void runOpMode() {
        // 初始化硬體 (只呼叫轉盤和 Encoder 所需的硬體)
        diskServo = hardwareMap.get(CRServo.class, "servo2");
        diskServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // 你的 Encoder 是插在 intakeMotor 的接口上
        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // 重置 Encoder 數值
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("=== 轉盤校正工具已就緒 ===");
        telemetry.addLine("操作說明：");
        telemetry.addLine("LB / RB : 慢速微調轉盤 (適合對齊)");
        telemetry.addLine("LT / RT : 快速轉動轉盤");
        telemetry.addLine("按 [X] 鍵 : 將當前位置歸零 (設為 POS_FILL_A)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = 0;

            // 1. 手動控制邏輯
            if (gamepad1.right_bumper) {
                power = 0.20; // 慢速正轉 (微調用)
            } else if (gamepad1.left_bumper) {
                power = -0.20; // 慢速反轉 (微調用)
            } else if (gamepad1.right_trigger > 0.1) {
                power = 0.6; // 快速正轉
            } else if (gamepad1.left_trigger > 0.1) {
                power = -0.6; // 快速反轉
            }

            diskServo.setPower(power);

            // 2. 歸零功能 (設定 Home 點)
            if (gamepad1.x) {
                intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // 3. 顯示當前數值
            int currentPos = intakeMotor.getCurrentPosition();

            telemetry.addLine("=== 轉盤校正模式 ===");
            telemetry.addData("當前 Encoder 數值", currentPos);
            telemetry.addLine(" ");
            telemetry.addLine("請依照以下步驟紀錄數值：");
            telemetry.addLine("1. 將 A 洞對準【進球口】，按 X 歸零 (此為 POS_FILL_A = 0)");
            telemetry.addLine("2. 轉動 A 洞直到對準【發射器】，抄下數值 (POS_FIRE_A)");
            telemetry.addLine("3. 轉動 B 洞直到對準【發射器】，抄下數值 (POS_FIRE_B)");
            telemetry.addLine("4. 轉動 C 洞直到對準【發射器】，抄下數值 (POS_FIRE_C)");
            telemetry.update();
        }
    }
}