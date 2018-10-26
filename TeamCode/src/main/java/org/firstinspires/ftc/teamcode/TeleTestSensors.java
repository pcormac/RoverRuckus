package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Cormac on 10/18/2017.
 */
@TeleOp(name = "Test Sensors", group = "Main")
public class TeleTestSensors extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Device declarations
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor elevator = null;
    //private DcMotor escalator = null;
    private Servo tail = null;
    private Servo gate = null;
    private Servo flip = null;

    private DigitalChannel elevatorTouch = null;
    private DigitalChannel mag = null;
    //TouchSensor escalatorTouch = null;
    //ColorSensor colorSensor = null;
    //ColorSensor colorSensorCloser = null;
    //ModernRoboticsI2cRangeSensor ultra = null;

    private double elevatorPower = 0;
    // double escalatorPower = 0;

    String color;

    private double tail_UP     = .75;
    private double tail_DOWN   = -.5;
    private double gate_CLOSED = -1;
    private double gate_OPEN   = .5;
    private double flip_UP     = -1;
    private double flip_DOWN   = .95;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // get motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        elevator = hardwareMap.dcMotor.get("elevator");
        //escalator = hardwareMap.dcMotor.get("escalator");

        // get servos
        tail = hardwareMap.servo.get("tail");
        gate = hardwareMap.servo.get("gate");
        flip = hardwareMap.servo.get("flip");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        //escalator.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorTouch = hardwareMap.get(DigitalChannel.class, "elevatorTouch");
        //escalatorTouch = hardwareMap.touchSensor.get("escalatorTouch");
        //colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        //colorSensorCloser = hardwareMap.get(ColorSensor.class, "colorSensorCloser");
        //ultra = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");

        mag = hardwareMap.get(DigitalChannel.class, "mag");
        elevatorTouch.setMode(DigitalChannel.Mode.INPUT);

        //colorSensor.enableLed(true);

        tail.setPosition(tail_UP);
        gate.setPosition(gate_CLOSED);
        flip.setPosition(flip_DOWN);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
        telemetry.addData("Right Encoder", -rightMotor.getCurrentPosition());
        telemetry.addData("Lift Encoder", elevator.getCurrentPosition());
        //telemetry.addData("Color", color);
        //telemetry.addData("raw ultrasonic", ultra.rawUltrasonic());
        telemetry.addData("Mag: ", !mag.getState());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}
