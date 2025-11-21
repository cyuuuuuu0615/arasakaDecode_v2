package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TestBenchColor {


    NormalizedColorSensor colorSensor;

    long PR = 0;
    long PG = 0;
    long PB = 0;
    long GR = 0;
    long GG = 0;
    long GB = 0;


    public enum DetectedColor {
        PURPLE,

        GREEN,

        UNKOWN
    }

    public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class,"colorSensor1");
    }

    public DetectedColor getDectedColor(Telemetry telemetry){
        NormalizedRGBA color = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = color.red / color.alpha;
        normGreen = color.green / color.alpha;
        normBlue = color.blue / color.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("bule", normBlue);

//        if(normRed > PR && normGreen > PG && normBlue > PB){
//            return DetectedColor.PURPLE;
//        }else if(normRed > GR && normGreen > GG && normBlue > GB){
//            return DetectedColor.GREEN;
//        }else {
//            return DetectedColor.UNKOWN;
//
//        }
        return DetectedColor.UNKOWN;


    }

}
