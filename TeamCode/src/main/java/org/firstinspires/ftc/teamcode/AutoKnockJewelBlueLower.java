package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Cormac on 9/30/2017.
 */

@Autonomous(name = "Auto: Knock Blue Lower", group = "Main")
public class AutoKnockJewelBlueLower extends AutoFunctions {
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

        hitColor("Blue");

        servoJewel.setPosition(1);

        sleep(1000);

        runUsingEncoders(.15, .15, 1000);

        if (vMark.equals("Left")) {
            turnSlowUsingEncoders("left", 550);
        } else if (vMark.equals("Center")) {
            turnSlowUsingEncoders("left", 575);
        } else if (vMark.equals("Right")) {
            turnSlowUsingEncoders("left", 600);
        } else {
            turnSlowUsingEncoders("left", 575);
        }
        sleep(1000);

        runUsingEncoders(.25, .25, 600);

        runForTime(.25, .25, 750);

        sleep(1000);

        openGrabbers();
        sleep(1000);

        runUsingEncoders(-.25, -.25, 200);
        elevatorDown();
        openGrabbers();
        runForTime(.25, .25, 1000);
        sleep(500);

        if (vMark.equals("Center")) {
            runForTime(.25, -.25, 250);
        }

        runUsingEncoders(-.25, -.25, 200);

        // force stop
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        telemetry.addData("Left Encoder: ", leftMotor.getCurrentPosition());
        telemetry.addData("Right Encoder: ", rightMotor.getCurrentPosition());
        telemetry.addData("Elevator Encoder: ", elevator.getCurrentPosition());
        telemetry.update();
        sleep(1000);
    }
}

