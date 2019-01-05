package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto: S Depot to Their Crater", group = "Depot Start")
public class AutoSDepotToTheirCrater extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    public void runOpMode() throws InterruptedException {

        declareMap();

        waitForStart();

        // get down from lander
        moveElevatorUsingEncoder(1, 5600);
        hook.setPower(-1);
        sleep(hookSleepTime);
        hook.setPower(0);
        elevatorDown();
        moveElevatorUsingEncoder(1, 500);

        runUsingRTP(400);
        turnUsingRTP("Left", 1700);
        runUsingRTP(150);

        checkDogeCVForTime(3);

        telemetry.addData("AutoStatus: ", "Starting back up");
        telemetry.update();
        runBackUsingRTP(250);

        if (gold.equalsIgnoreCase("Center")) {
            turnUsingRTP("Left", 50);
            runBackUsingRTP(2500);
        } else if (gold.equalsIgnoreCase("Right")) {
            turnUsingRTP("Right", 450);
            runBackUsingRTP(1500);
            turnUsingRTP("Left", 600);
            runBackUsingRTP(1500);
            turnUsingRTP("Left", 200);
        } else {
            // default to left
            turnUsingRTP("Left", 400);
            runBackUsingRTP(2000);
            turnUsingRTP("Right", 600);
            runBackUsingRTP(1500);
            turnUsingRTP("Right", 200);
        }


        telemetry.addData("AutoStatus: ", "Dumping marker");
        telemetry.update();

        double oldRuntime = runtime.seconds();
        while (runtime.seconds() < (oldRuntime + 2.0)) {
            // starts the position at tail_UP then takes the progress as a percent of the time it has been running
            // then multiplies that % by the total difference in position from up to down
            tail.setPosition(tail_UP - (runtime.seconds()/(oldRuntime + 2.0))*(tail_UP - tail_DOWN));
            idle();
        }
        tail.setPosition(tail_DOWN);
        sleep(1000);

        moveElevatorUsingEncoder(1, 500);

        if (gold.equalsIgnoreCase("Left")) {
            turnUsingRTP("Right", 100);
            runUsingRTP(3750);
        } else if (gold.equalsIgnoreCase("Center")) {
            runBackUsingRTP(300);
            turnUsingRTP("Right", 300);
            runUsingRTP(3750);
        } else {
            turnUsingRTP("Left", 100);
            runUsingRTP(3500);
        }

        tail.setPosition(tail_UP);
        telemetry.addData("AutoStatus: ", "Done");
        telemetry.update();
        detector.disable();
    }
}



