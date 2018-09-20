package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Cormac on 10/18/2017.
 */
@Autonomous(name = "Auto: Test Encoders", group = "Main")
public class AutoTestEncoders extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    public void runOpMode() throws InterruptedException {

        declareMap();

        servoJewel.setPosition(1);

        waitForStart();

        runForTime(1, 1, 500);

        telemetry.addData("Left: ", leftMotor.getCurrentPosition());
        telemetry.addData("Right: ", -rightMotor.getCurrentPosition());
        telemetry.update();

        sleep(5000);

        runUsingEncoders(.25, .25, 1000);

        sleep(3000);

        turnUsingEncoders("left", 500);

        sleep(3000);
    }
}
