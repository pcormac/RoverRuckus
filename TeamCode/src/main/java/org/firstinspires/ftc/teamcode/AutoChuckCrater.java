package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto: Chuck Crater", group = "Main")
public class AutoChuckCrater extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    public void runOpMode() throws InterruptedException {

        declareMap();

        waitForStart();

        elevator.setPower(1);

        sleep(250);

        elevator.setPower(0);

        telemetry.addData("AutoStatus: ", "Starting back up");
        telemetry.update();
        runUsingEncoders(-.5, -.5, 1400);


        telemetry.addData("AutoStatus: ", "Dumping marker");
        telemetry.update();
        tail.setPosition(tail_DOWN);
        sleep(500);
        tail.setPosition(tail_UP);


        telemetry.addData("AutoStatus: ", "Driving to crater");
        telemetry.update();
        turnSlowUsingEncoders("Left", 300);
        runUsingEncoders(.5, .5, 1800);

        telemetry.addData("AutoStatus: ", "Done");
        telemetry.update();
    }
}

