package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto: Depot Mark Crater", group = "Depot Start")
public class AutoDepotMarkCrater extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    public void runOpMode() throws InterruptedException {

        declareMap();

        waitForStart();

        elevator.setPower(1);

        sleep(500);

        elevator.setPower(0);

        telemetry.addData("AutoStatus: ", "Starting back up");
        telemetry.update();
        runUsingEncoders(-.5, -.5, 1000);
        turnUsingEncoders("Left", 300);
        runUsingEncoders(-.5, -.5, 300);
        runUsingEncoders(.5, .5, 300);
        turnUsingEncoders("Right", 300);
        runUsingEncoders(-.5, -.5, 1000);


        telemetry.addData("AutoStatus: ", "Dumping marker");
        telemetry.update();
        tail.setPosition(tail_DOWN);
        sleep(500);
        tail.setPosition(tail_UP);


        telemetry.addData("AutoStatus: ", "Driving to crater");
        telemetry.update();
        turnSlowUsingEncoders("Left", 300);
        runUsingEncoders(.75, .75, 3250);

        telemetry.addData("AutoStatus: ", "Done");
        telemetry.update();
    }
}

