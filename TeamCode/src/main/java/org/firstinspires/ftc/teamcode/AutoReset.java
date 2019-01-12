package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Cormac on 12/6/2016.
 * Purely for quickly reseting autonomous for testing
 */
@Autonomous(name="Auto: Reset", group="Misc")
public class AutoReset extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    public void runOpMode() throws InterruptedException {

        declareMap();

        waitForStart();

        runtime.reset();

        while (opModeIsActive() && !isStopRequested() && runtime.seconds() < ((hookSleepTime / 1000) + .05)) {
            if (elevatorTouch.getState()) {
                leftElevator.setPower(-1);
                rightElevator.setPower(-1);
                hook.setPower(1);
            } else {
                leftElevator.setPower(0);
                rightElevator.setPower(0);
                hook.setPower(1);
            }
            idle();
        }

        hook.setPower(0);
        leftElevator.setPower(0);
        rightElevator.setPower(0);
        sleep(250);

        telemetry.addData("AutoStatus: ", "Reset done");
        telemetry.update();
    }

    // new code


}
