package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Cormac on 11/21/2017.
 */
@Autonomous(name = "Vuforia: Test", group = "Vuforia")
public class VuforiaTest extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    public void runOpMode() throws InterruptedException {
        declareMap();

        waitForStart();

        checkVuforiaForTime(3);
        telemetry.addData("Vuforia: ", vMark);
        telemetry.update();
        sleep(5000);
    }
}
