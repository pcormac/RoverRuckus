package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SilverDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="TeleRightTilt", group="DogeCV")
public class TeleRightTiltMineral extends OpMode {

    private TwoMineralSampleDetector detector;
    private List<String> tempScorerNames = new ArrayList<>();

    @Override
    public void init() {
        detector = new TwoMineralSampleDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.downscale = 0.4;
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        detector.goldPerfectAreaScorer.weight = 0.01;
        detector.goldPerfectAreaScorer.weight = 0.01;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        telemetry.addData("Order", detector.getCurrentOrder());
        telemetry.update();
    }

    @Override
    public void stop() {
        detector.disable();
    }
}
