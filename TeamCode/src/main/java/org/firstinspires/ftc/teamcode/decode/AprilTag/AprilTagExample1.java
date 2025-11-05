package org.firstinspires.ftc.teamcode.decode.AprilTag;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagExample1 extends OpMode {

    ApriltagWebcam apriltagWebcam = new ApriltagWebcam();

    @Override
    public void init(){
        apriltagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        apriltagWebcam.update();
        AprilTagDetection id20 = apriltagWebcam.getTagBySpecificId(20);
        apriltagWebcam.displayDecteionTelemetry(id20);
        telemetry.addData("id20 String", id20.toString());
    }

}
