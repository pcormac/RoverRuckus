package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto: Chuck Marker", group = "Main")
public class AutoChuckMarker extends AutoFunctions {
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

        runUsingEncoders(-.5, -.5, 1650);

        tail.setPosition(tail_DOWN);
        sleep(500);

        runUsingEncoders(.5, .5, 600);

        tail.setPosition(tail_UP);

        telemetry.addData("AutoStatus: ", "Done");
        telemetry.update();
    }

    // new code


}

