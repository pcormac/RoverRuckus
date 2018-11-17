package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto: Sample Depot", group = "Depot Start")
public class AutoSampleDepot extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    public void runOpMode() throws InterruptedException {

        declareMap();

        waitForStart();

        moveElevatorUsingEncoder(1, 500);

        checkDogeCVForTime(5);

        telemetry.addData("AutoStatus: ", "Starting back up");
        telemetry.update();

        if (gold.equalsIgnoreCase("Left")) {
            turnUsingEncoders("Left", 80);
            runUsingEncoders(-.5, -.5,1500);
            turnUsingEncoders("Right", 250);
            runUsingEncoders(-.5, -.5,1000);
        } else if (gold.equalsIgnoreCase("Middle")) {
            turnUsingEncoders("Right", 35);
            runUsingEncoders(-.5, -.5,2000);
        } else if (gold.equalsIgnoreCase("Right")) {
            turnUsingEncoders("Right", 225);
            runUsingEncoders(-.5, -.5,1200);
            turnUsingEncoders("Left", 200);
            runUsingEncoders(-.5, -.5,1200);
        } else {
            // mineral was not found, default to right
            turnUsingEncoders("Right", 225);
            runUsingEncoders(-.5, -.5,1200);
            turnUsingEncoders("Left", 200);
            runUsingEncoders(-.5, -.5,1200);
        }


        telemetry.addData("AutoStatus: ", "Dumping marker");
        telemetry.update();
        tail.setPosition(tail_DOWN);
        sleep(500);

        moveElevatorUsingEncoder(1, 500);

        if (gold.equalsIgnoreCase("Left")) {
            turnUsingEncoders("Right", 100);
            runUsingEncoders(3500);
        } else if (gold.equalsIgnoreCase("Middle")) {
            runUsingEncoders(-.5, -.5, 300);
            turnUsingEncoders("Right", 300);
            runUsingEncoders(3750);
        } else {
            turnUsingEncoders("Left", 100);
            runUsingEncoders(3500);
        }

        tail.setPosition(tail_UP);
        telemetry.addData("AutoStatus: ", "Done");
        telemetry.update();
    }
}


