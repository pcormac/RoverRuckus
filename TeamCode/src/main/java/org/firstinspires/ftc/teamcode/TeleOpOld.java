package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpOld", group="Iterative Opmode")
public class TeleOpOld extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Device declarations
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor elevator = null;
    private Servo tail = null;
    private Servo gate = null;
    private Servo flip = null;

    private DigitalChannel elevatorTouch = null;
    private DigitalChannel mag = null;

    private double elevatorPower = 0;

    private double tail_UP     = .75;
    private double tail_DOWN   = -.5;
    private double gate_CLOSED = -1;
    private double gate_OPEN   = .5;
    private double flip_UP     = -1;
    private double flip_DOWN   = .95;

    private boolean magMove;
    private boolean magOn;
    double currTime;
    double goalTime = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // get motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        elevator = hardwareMap.dcMotor.get("elevator");

        // get servos
        tail = hardwareMap.servo.get("tail");
        gate = hardwareMap.servo.get("gate");
        flip = hardwareMap.servo.get("flip");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD);

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

        mag = hardwareMap.get(DigitalChannel.class, "mag");
        elevatorTouch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {
        tail.setPosition(tail_UP);
        gate.setPosition(gate_CLOSED);
        flip.setPosition(flip_DOWN);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString() + ", " + runtime.seconds());
        currTime = runtime.seconds();
        magOn = !mag.getState();

        /* Controls
        Game pad 1
        Left Stick    = Left drive
        Right Stick   = Right Drive
        A             = Open gate
        B             =
        Y             = Drop tail
        X             =
        Left Bumper   = .25 speed
        Right Bumper  =
        Game pad 2
        Left Stick    = Y:Elevator
        Right Stick   =
        A             = Elevator up using mag sensor
        B             = Elevator down using mag sensor
        Y             = Flip up
        X             =
        Left Bumper   =
        Right Bumper  =
        */


        // Tank drive
        if (gamepad1.left_stick_y < .25 && gamepad1.left_stick_y > 0) {
            leftMotor.setPower(0);
        } else if (gamepad1.left_stick_y > -.25 && gamepad1.left_stick_y < 0) {
            leftMotor.setPower(0);
        } else if (gamepad1.dpad_up) {
            leftMotor.setPower(1);
        }
        else if (gamepad1.dpad_down) {
            leftMotor.setPower(-1);
        }
        else {
            if (gamepad1.left_bumper) {
                leftMotor.setPower(.5*-gamepad1.left_stick_y);
            }
            else {
                leftMotor.setPower(-gamepad1.left_stick_y);
            }
        }

        if (gamepad1.right_stick_y < .25 && gamepad1.right_stick_y > 0) {
            rightMotor.setPower(0);
        } else if (gamepad1.right_stick_y > -.25&& gamepad1.right_stick_y < 0) {
            rightMotor.setPower(0);
        } else if (gamepad1.dpad_up) {
            rightMotor.setPower(1);
        }
        else if (gamepad1.dpad_down) {
            rightMotor.setPower(-1);
        }
        else {
            if (gamepad1.left_bumper) {
                rightMotor.setPower(.5*gamepad1.right_stick_y);
            }
            else {
                rightMotor.setPower(gamepad1.right_stick_y);
            }
        }

        // Drop tail
        if (gamepad1.y) {
            // down
            tail.setPosition(tail_DOWN);
        } else {
            // up
            tail.setPosition(tail_UP);
        }

        // gate
        if (gamepad1.a) {
            // open
            gate.setPosition(gate_OPEN);
        } else {
            // close
            gate.setPosition(gate_CLOSED);
        }

        //
        //        JOY 2
        //


        if (gamepad2.b) {
            // use mag to go up
            goalTime = currTime + .5;
            elevatorPower = 1;
            magMove = true;
        } else if (gamepad2.a) {
            // use mag to go down
            goalTime = currTime + .5;
            elevatorPower = -1;
            magMove = true;
        }

        if (magOn && (currTime > goalTime)) {
            magMove = false;
            elevatorPower = 0;
        }

        // elevator
        if (Math.abs(gamepad2.left_stick_y) > .25) {
            elevator.setPower(-gamepad2.left_stick_y);
        } else if (magMove && elevatorTouch.getState()) {
            // for ele touch true means that it isn't touched
            if (currTime < goalTime) {
                elevator.setPower(elevatorPower);
            } else if (!magOn){
                elevator.setPower(elevatorPower);
            }
        } else if (!elevatorTouch.getState()) {
            magMove = false;
            elevatorPower = 0;
            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevator.setPower(.5);
        } else {
            elevator.setPower(0);
        }

        // flip
        if (gamepad2.y) {
            flip.setPosition(flip_UP);
        } else {
            flip.setPosition(flip_DOWN);
        }

        // end of code, update telemetry
        telemetry.addData("Right", gamepad1.right_stick_y);
        telemetry.addData("Left", gamepad1.left_stick_y);
        telemetry.addData("Slow", gamepad1.left_bumper);
        telemetry.addData("Elevator", elevator.getPower());
        telemetry.addData("Elevator Touch", elevatorTouch.getState());
        telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
        telemetry.addData("Right Encoder", -rightMotor.getCurrentPosition());
        telemetry.addData("Lift Encoder", elevator.getCurrentPosition());
        telemetry.addData("Mag", !mag.getState());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}