package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto: Just Crater", group = "Main")
public class AutoJustCrater extends AutoFunctions {
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

        runUsingEncoders(-.5, -.5, 1750);

        tail.setPosition(tail_DOWN);
        sleep(500);
        tail.setPosition(tail_UP);

        telemetry.addData("AutoStatus: ", "Done");
        telemetry.update();
    }
}

