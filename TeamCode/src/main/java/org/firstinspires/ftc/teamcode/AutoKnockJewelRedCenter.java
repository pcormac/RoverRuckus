package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Cormac on 11/6/2017.
 */
@Autonomous(name = "Auto: Knock Red Center", group = "Main")
public class AutoKnockJewelRedCenter extends AutoFunctions {
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
        checkVuforiaForTime(3.0);

        moveElevatorUsingEncoder(1, 1750);

        servoJewel.setPosition(jewelDown); // slice the balls
        // color sensor is on the left (from the robot arm's view) of the slicer, robot is backwards
        getColor();

        hitColor("Red");

        servoJewel.setPosition(1);

        sleep(1000);

        if (vMark.equals("Left")) {
            turnSlowUsingEncoders("right", 550);
        } else if (vMark.equals("Center")) {
            turnSlowUsingEncoders("right", 575);
        } else if (vMark.equals("Right")) {
            turnSlowUsingEncoders("right", 600);
        } else {
            turnSlowUsingEncoders("right", 575);
        }
        sleep(1000);

        runUsingEncoders(1100);

        sleep(1000);

        turnSlowUsingEncoders("right", 450);

        runForTime(.5, .5, 500);

        openGrabbers();
        sleep(1000);

        runUsingEncoders(-.25, -.25, 200);
        sleep(500);

        elevatorDown();
        openGrabbers();

        runForTime(.25, .25, 600);

        runUsingEncoders(-.25, -.25, 200);

        // force stop
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        telemetry.addData("Left Encoder: ", leftMotor.getCurrentPosition());
        telemetry.addData("Right Encoder: ", rightMotor.getCurrentPosition());
        telemetry.addData("Elevator Encoder: ", elevator.getCurrentPosition());
        telemetry.update();
        sleep(5000);
    }
}


