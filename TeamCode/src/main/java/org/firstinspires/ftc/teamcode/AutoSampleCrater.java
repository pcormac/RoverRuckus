package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto: Sample Crater", group = "Crater Start")
public class AutoSampleCrater extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    public void runOpMode() throws InterruptedException {

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
        } else if (gold.equalsIgnoreCase("Right")) {
            turnUsingRTP("Right", 425);
            runBackUsingRTP(1500);
            turnUsingRTP("Left", 600);
            runBackUsingRTP(1500);
        } else {
            // default to left
            gold = "Left";
            turnUsingRTP("Left", 400);
            runBackUsingRTP(2000);
            turnUsingRTP("Right", 800);
            runBackUsingRTP(1500);
        }

        telemetry.addData("AutoStatus: ", "Done");
        telemetry.update();
    }
}

