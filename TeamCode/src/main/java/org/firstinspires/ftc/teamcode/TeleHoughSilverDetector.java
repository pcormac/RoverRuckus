package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.HoughSilverDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Size;

@Disabled
@TeleOp(name="Hough Silver Example", group="DogeCV")
public class TeleHoughSilverDetector extends OpMode {
    //Detector object
    private HoughSilverDetector detector;

    @Override
    public void init() {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold SilverDetector Example");

        detector = new HoughSilverDetector(); //Create detector
        detector.downscale = 1; //Increase detector sensitivity with smaller size. Make sure to preserve aspect ratio.
        detector.useFixedDownscale = false; //Don't fix the downscale
        detector.sensitivity = 1.6; //Play with this based on your camera, adjusts how sensitive the detector is
        detector.minDistance = 60; //Minimum distance between silver mineral centers in pixels
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); //Initialize detector with app context and camera
        detector.useDefaults(); //Use default settings

        // Optional Tuning
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.enable(); //Start the detector
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {}

    @Override
    public void stop() {
        detector.disable();
    }

}
