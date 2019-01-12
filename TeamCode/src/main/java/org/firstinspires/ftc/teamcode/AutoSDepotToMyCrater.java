package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Auto: S Depot to My Crater", group = "Depot Start")
public class AutoSDepotToMyCrater extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    public void runOpMode() throws InterruptedException {

        declareMap();

        waitForStart();

        // get down from lander
        moveElevatorUsingEncoder(1, 5700);
        hook.setPower(-1);
        sleep(hookSleepTime);
        hook.setPower(0);
        elevatorDown();
        moveElevatorUsingEncoder(1, 500);

        runUsingRTP(400);
        turnUsingRTP("Left", 1715);
        runUsingRTP(150);

        checkDogeCVForTime(3);

        telemetry.addData("AutoStatus: ", "Starting back up");
        telemetry.update();
        runBackUsingRTP(250);

        if (gold.equalsIgnoreCase("Center")) {
            turnUsingRTP("Left", 100);
            runBackUsingRTP(2500);
            turnUsingRTP("Right", 150);
        } else if (gold.equalsIgnoreCase("Right")) {
            turnUsingRTP("Right", 425);
            runBackUsingRTP(1500);
            turnUsingRTP("Left", 600);
            runBackUsingRTP(1500);
            turnUsingRTP("Left", 200);
        } else {
            // default to left
            gold = "Left";
            turnUsingRTP("Left", 400);
            runBackUsingRTP(2000);
            turnUsingRTP("Right", 800);
            runBackUsingRTP(1500);
        }

        telemetry.addData("AutoStatus: ", "Dumping marker");
        telemetry.update();

        double oldRuntime = runtime.seconds();
        while (runtime.seconds() < (oldRuntime + 2.0)) {
            // starts the position at tail_UP then takes the progress as a percent of the time it has been running
            // then multiplies that % by the total difference in position from up to down
            tail.setPosition(tail_UP - ((runtime.seconds() - oldRuntime)/(2.0))*(tail_UP - tail_DOWN));
            idle();
        }
        tail.setPosition(tail_UP);

        if (gold.equalsIgnoreCase("Left")) {
            runUsingRTP(300);
            turnUsingRTP("Left", 1400);
            runUsingRTP(700);
            turnUsingRTP("Right", 300);
            runUsingRTP(700);
            turnUsingRTP("Right", 225);
            runUsingRTP(2500, 1);
        } else if (gold.equalsIgnoreCase("Center")) {
            runBackUsingRTP(500);
            turnUsingRTP("Left", 750);
            runUsingRTP(1000);
            turnUsingRTP("Right", 225);
            runUsingRTP(2500, 1);
        } else if (gold.equalsIgnoreCase("Right")) {
            turnUsingRTP("Left", 250);
            runUsingRTP(3000, 1);
        }

        if ((distance.getDistance(DistanceUnit.INCH) < 20) && (distance.getDistance(DistanceUnit.INCH) > 7)) {
            turnUsingRTP("Left", 200);
        } else if ((distance.getDistance(DistanceUnit.INCH) < 20) && (distance.getDistance(DistanceUnit.INCH) > 5)) {
            turnUsingRTP("Left", 100);
        }
        runUsingRTP(1000);

        tail.setPosition(tail_UP);
        telemetry.addData("AutoStatus: ", "Done");
        telemetry.update();
        detector.disable();
    }
}



