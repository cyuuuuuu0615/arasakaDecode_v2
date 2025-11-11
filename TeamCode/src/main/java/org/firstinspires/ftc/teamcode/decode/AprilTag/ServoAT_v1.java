package org.firstinspires.ftc.teamcode.decode.AprilTag;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Advanced Simple Tracker", group = "Competition")
public class ServoAT_v1 extends LinearOpMode {

    private Servo panServo;
    private Servo tiltServo;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // 伺服馬達參數
    private static final double PAN_MIN = 0.0;
    private static final double PAN_MAX = 1.0;
    private static final double PAN_CENTER = 0.5;

    private static final double TILT_MIN = 0.2;
    private static final double TILT_MAX = 0.8;
    private static final double TILT_CENTER = 0.5;

    // 追蹤參數
    private static final double PAN_SPEED = 0.015;
    private static final double TILT_SPEED = 0.01;
    private static final double DEAD_ZONE = 0.03;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Status", "Initialized - Ready for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            trackNearestAprilTag();
            telemetry.update();
            sleep(20);
        }
    }

    private void initializeHardware() {
        // 初始化伺服馬達
        panServo = hardwareMap.get(Servo.class, "panServo");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");

        // 重置到中心位置
        panServo.setPosition(PAN_CENTER);
        tiltServo.setPosition(TILT_CENTER);

        // 初始化 AprilTag
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    private void trackNearestAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.size() > 0) {
            // 找到最近的 AprilTag
            AprilTagDetection nearestTag = findNearestTag(detections);

            if (nearestTag != null) {
                trackTag(nearestTag);
            }
        } else {
            telemetry.addData("Status", "WAITING FOR TAG");
            telemetry.addData("Pan Position", "%.3f", panServo.getPosition());
            telemetry.addData("Tilt Position", "%.3f", tiltServo.getPosition());
        }
    }

    private AprilTagDetection findNearestTag(List<AprilTagDetection> detections) {
        AprilTagDetection nearest = null;
        double minDistance = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose.range < minDistance) {
                minDistance = detection.ftcPose.range;
                nearest = detection;
            }
        }

        return nearest;
    }

    private void trackTag(AprilTagDetection detection) {
        // 水平追蹤 (Pan)
        double panError = calculatePanError(detection);
        if (Math.abs(panError) > DEAD_ZONE) {
            double newPanPosition = panServo.getPosition() - (panError * PAN_SPEED);
            newPanPosition = clamp(newPanPosition, PAN_MIN, PAN_MAX);
            panServo.setPosition(newPanPosition);
        }

        // 垂直追蹤 (Tilt) - 可選功能
        double tiltError = calculateTiltError(detection);
        if (Math.abs(tiltError) > DEAD_ZONE) {
            double newTiltPosition = tiltServo.getPosition() - (tiltError * TILT_SPEED);
            newTiltPosition = clamp(newTiltPosition, TILT_MIN, TILT_MAX);
            tiltServo.setPosition(newTiltPosition);
        }

        // 顯示追蹤資訊
        telemetry.addData("Status", "TRACKING TAG #" + detection.id);
        telemetry.addData("Distance", "%.1f inches", detection.ftcPose.range);
        telemetry.addData("Pan Error", "%.3f", panError);
        telemetry.addData("Tilt Error", "%.3f", tiltError);
        telemetry.addData("Pan Position", "%.3f", panServo.getPosition());
        telemetry.addData("Tilt Position", "%.3f", tiltServo.getPosition());
    }

    private double calculatePanError(AprilTagDetection detection) {
        // 使用 bearing 來計算水平誤差
        // bearing: 負值在左邊，正值在右邊
        return detection.ftcPose.bearing / 45.0; // 歸一化到合理範圍
    }

    private double calculateTiltError(AprilTagDetection detection) {
        // 使用 elevation 來計算垂直誤差
        // elevation: 負值在下方，正值在上方
        return detection.ftcPose.elevation / 30.0; // 歸一化到合理範圍
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}