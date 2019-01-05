package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.HSVRangeFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.DogeCVScorer;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

// This program is a modified version of DogeCVs SamplingOrderDetector program adapted to conclude the gold's position
// given only the left and center minerals.

public class TwoMineralSampleDetector extends DogeCVDetector {

    public enum GoldLocation {
        UNKNOWN,
        LEFT,
        CENTER,
        RIGHT
    }

    private List<DogeCVScorer> goldScorers = new ArrayList<>();
    private List<DogeCVScorer> silverScorers = new ArrayList<>();
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;

    //Create the scorers used for the detector
    public RatioScorer ratioScorer                   = new RatioScorer(1.0,5);
    public MaxAreaScorer maxAreaScorer               = new MaxAreaScorer(0.01);
    public PerfectAreaScorer goldPerfectAreaScorer   = new PerfectAreaScorer(2500,0.01);
    public PerfectAreaScorer silverPerfectAreaScorer = new PerfectAreaScorer(1500,0.01);

    //Create the filters used
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW,100);
    public DogeCVColorFilter whiteFilter  = new HSVRangeFilter(new Scalar(0,0,200), new Scalar(50,40,255));


    // Results for the detector
    private TwoMineralSampleDetector.GoldLocation currentOrder = TwoMineralSampleDetector.GoldLocation.UNKNOWN;
    private TwoMineralSampleDetector.GoldLocation lastOrder    = TwoMineralSampleDetector.GoldLocation.UNKNOWN;
    private boolean isFound = false;

    // Create the mats used
    private Mat workingMat  = new Mat();
    private Mat displayMat  = new Mat();
    private Mat yellowMask  = new Mat();
    private Mat whiteMask   = new Mat();
    private Mat hiarchy     = new Mat();

    public TwoMineralSampleDetector() {
        super();
        this.detectorName = "Sampling Order Detector";
    }

    @Override
    public Mat process(Mat input) {

        // Copy input mat to working/display mats
        input.copyTo(displayMat);
        input.copyTo(workingMat);
        input.release();

        // Generate Masks
        yellowFilter.process(workingMat.clone(), yellowMask);
        whiteFilter.process(workingMat.clone(), whiteMask);


        // Blur and find the contours in the masks
        List<MatOfPoint> contoursYellow = new ArrayList<>();
        List<MatOfPoint> contoursWhite = new ArrayList<>();

        Imgproc.blur(whiteMask,whiteMask,new Size(2,2));
        Imgproc.blur(yellowMask,yellowMask,new Size(2,2));

        Imgproc.findContours(yellowMask, contoursYellow, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(230,70,70),2);

        Imgproc.findContours(whiteMask, contoursWhite, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursWhite,-1,new Scalar(230,70,70),2);


        // Prepare to find best yellow (gold) results
        Rect chosenYellowRect  = null;
        double chosenYellowScore = Integer.MAX_VALUE;

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        for(MatOfPoint c : contoursYellow){
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            double differenceScore = calculateScore(points, "gold");

            if(differenceScore < chosenYellowScore && differenceScore < maxDifference){
                chosenYellowScore = differenceScore;
                chosenYellowRect = rect;
            }

            double area = Imgproc.contourArea(c);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));
            if((area > 1000) && (area < 5000)){
                Imgproc.circle(displayMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(displayMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
            }
        }

        // Prepare to find best white (silver) results
        List<Rect>   chosenWhiteRect  = new ArrayList<>(2);
        List<Double> chosenWhiteScore  = new ArrayList<>(2);
        chosenWhiteScore.add(0, Double.MAX_VALUE);
        chosenWhiteScore.add(1, Double.MAX_VALUE);
        chosenWhiteRect.add(0, null);
        chosenWhiteRect.add(1, null);


        for(MatOfPoint c : contoursWhite){
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            double differenceScore = calculateScore(points, "silver");

            double area = Imgproc.contourArea(c);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));
            if((area > 500) && (area < 5000)){
                Imgproc.circle(displayMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(displayMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
                Imgproc.putText(displayMat,"Diff: " + differenceScore,new Point(centerPoint.x, centerPoint.y + 20),0,0.5,new Scalar(0,255,255));
            }

            boolean good = true;
            if(differenceScore < maxDifference && area > 1000){

                if(differenceScore < chosenWhiteScore.get(0)){
                    chosenWhiteRect.set(0,rect);
                    chosenWhiteScore.set(0,differenceScore);
                }
                else if(differenceScore < chosenWhiteScore.get(1) && differenceScore > chosenWhiteScore.get(0)){
                    chosenWhiteRect.set(1,rect);
                    chosenWhiteScore.set(1, differenceScore);
                }
            }


        }

        //Draw found gold element
        if(chosenYellowRect != null){
            Imgproc.rectangle(displayMat,
                    new Point(chosenYellowRect.x, chosenYellowRect.y),
                    new Point(chosenYellowRect.x + chosenYellowRect.width, chosenYellowRect.y + chosenYellowRect.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(displayMat,
                    "Gold: " + String.format("%.2f X=%.2f", chosenYellowScore, (double)chosenYellowRect.x),
                    new Point(chosenYellowRect.x - 5, chosenYellowRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    2.0,
                    new Scalar(0, 255, 255),
                    4);

        }
        //Draw found white elements
        for(int i=0;i<chosenWhiteRect.size();i++){
            Rect rect = chosenWhiteRect.get(i);
            if(rect != null){
                double score = chosenWhiteScore.get(i);
                Imgproc.rectangle(displayMat,
                        new Point(rect.x, rect.y),
                        new Point(rect.x + rect.width, rect.y + rect.height),
                        new Scalar(255, 255, 255), 2);
                Imgproc.putText(displayMat,
                        "Silver: " + String.format("Score %.2f ", score) ,
                        new Point(rect.x - 5, rect.y - 10),
                        Core.FONT_HERSHEY_PLAIN,
                        2.0,
                        new Scalar(0, 255, 255),
                        4);
            }


        }

        // If enough elements are found, compute gold position
        if ((chosenYellowRect != null && chosenWhiteRect.get(0) != null) && chosenYellowRect.y < 500) {
            if (chosenYellowRect.x > chosenWhiteRect.get(0).x) {
                currentOrder = GoldLocation.RIGHT;
            } else {
                currentOrder = GoldLocation.CENTER;
            }
        } else if (chosenWhiteRect.get(0) != null && chosenWhiteRect.get(1) != null) {
            currentOrder = GoldLocation.LEFT;
        } else if (chosenYellowRect != null && chosenYellowRect.y < 600) {
            if (chosenYellowRect.x > 240) {
                currentOrder = GoldLocation.RIGHT;
            } else {
                currentOrder = GoldLocation.CENTER;
            }
        } else {
            currentOrder = GoldLocation.UNKNOWN;
            isFound = false;
        }

        //Display Debug Information
        Imgproc.putText(displayMat,"Gold Position: " + lastOrder.toString(),new Point(10,getAdjustedSize().height - 30),0,1, new Scalar(255,255,0),1);
        Imgproc.putText(displayMat,"Current Track: " + currentOrder.toString(),new Point(10,getAdjustedSize().height - 10),0,0.5, new Scalar(255,255,255),1);

        return displayMat;
    }

    @Override
    public void useDefaults() {
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            goldScorers.add(maxAreaScorer);
            silverScorers.add(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            goldScorers.add(goldPerfectAreaScorer);
            silverScorers.add(silverPerfectAreaScorer);
        }
        goldScorers.add(ratioScorer);
        silverScorers.add(ratioScorer);
    }

    /**
     * Is both elements found?
     * @return if the elements are found
     */
    public boolean isFound() {
        return isFound;
    }

    /**
     * Returns the current gold pos
     * @return current gold pos (UNKNOWN, LEFT, CENTER, RIGHT)
     */
    public GoldLocation getCurrentOrder() {
        return currentOrder;
    }

    /**
     * Returns the last known gold pos
     * @return last known gold pos (UNKNOWN, LEFT, CENTER, RIGHT)
     */
    public GoldLocation getLastOrder() {
        return lastOrder;
    }

    public double calculateScore(Mat input, String color) {
        double totalScore = 0;

        if (color.equalsIgnoreCase("gold")) {
            for(DogeCVScorer scorer : goldScorers){
                totalScore += scorer.calculateScore(input);
            }
        } else {
            for(DogeCVScorer scorer : silverScorers){
                totalScore += scorer.calculateScore(input);
            }
        }


        return totalScore;
    }
}