package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Cormac on 10/14/2017.
 */
@Autonomous(name = "Auto: Just Knock Blue", group = "Main")
public class AutoJustKnockBlue extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    public void runOpMode() throws InterruptedException {

        declareMap();

        servoJewel.setPosition(1); // upper position

        waitForStart();

        closeGrabbers();
        sleep(1000);

        moveElevatorUsingEncoder(1, 1375);

        servoJewel.setPosition(jewelDown); // slice the balls
        // color sensor is on the left (from the robot arm's view) of the slicer, robot is backwards
        getColor();

        sleep(1000);

        hitColor("Blue");

        servoJewel.setPosition(1);

        // force stop
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        elevatorDown();

        sleep(5000);
    }
}

