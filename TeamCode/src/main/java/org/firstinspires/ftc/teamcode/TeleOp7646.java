/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTelemetry;

@TeleOp(name="TeleOp7646", group="Iterative Opmode")
public class TeleOp7646 extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Device declarations
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor leftElevator = null;
    private DcMotor rightElevator = null;
    private DcMotorSimple escalator = null;
    private DcMotorSimple hook = null;
    //private DcMotorSimple flip = null;
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
    private double leftElePow = 0;
    private double rightElePow = 0;
    double escalatorPower = 0;

    String color;

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

    private boolean ohNoFound;
    private boolean wasRB = false;
    private int ohNoID;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // get motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftElevator = hardwareMap.dcMotor.get("leftElevator");
        rightElevator = hardwareMap.dcMotor.get("rightElevator");
        escalator = hardwareMap.get(DcMotorSimple.class, "escalator");
        hook = hardwareMap.get(DcMotorSimple.class, "hook");

        // get servos
        tail = hardwareMap.servo.get("tail");
        gate = hardwareMap.servo.get("gate");
        flip = hardwareMap.servo.get("flip");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftElevator.setDirection(DcMotor.Direction.FORWARD);
        rightElevator.setDirection(DcMotor.Direction.FORWARD);
        //escalator.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorTouch = hardwareMap.get(DigitalChannel.class, "elevatorTouch");
        //escalatorTouch = hardwareMap.touchSensor.get("escalatorTouch");
        //colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        //colorSensorCloser = hardwareMap.get(ColorSensor.class, "colorSensorCloser");
        //ultra = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");

        mag = hardwareMap.get(DigitalChannel.class, "mag");
        elevatorTouch.setMode(DigitalChannel.Mode.INPUT);

        //colorSensor.enableLed(true);
        int ohNoID = hardwareMap.appContext.getResources().getIdentifier("ohno",   "raw", hardwareMap.appContext.getPackageName());
        if (ohNoID != 0) {
            ohNoFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, ohNoID);
        }
        telemetry.addData("ohNo resource",   ohNoFound ?   "Found" : "NOT found\n Add ohno.wav to /src/main/res/raw" );

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
        B             = Move Hook Left
        Y             = Drop tail
        X             = Move Hook Right
        Left Bumper   = .25 speed
        Right Bumper  =

        Game pad 2
        Left Stick    = Y:Elevator X:Escalator
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

        // hook
        if (gamepad1.b) {
            hook.setPower(1);
        } else if (gamepad1.x) {
            hook.setPower(-1);
        } else {
            hook.setPower(0);
        }

        /*
        if (gamepad1.right_bumper && !wasRB && ohNoFound) {
            try {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, ohNoID);
                wasRB = !wasRB;
            } catch (Exception e) {
                telemetry.addData("Sound", "Failed");
                telemetry.update();
            }
        }
        */

        //
        //        JOY 2
        //

        /* Commented out until mag sensors are on new robot
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
        */

        if (magOn && (currTime > goalTime)) {
            magMove = false;
            elevatorPower = 0;
        }

        // elevator
        if (Math.abs(gamepad2.left_stick_y) > .25) {
            moveElevator(-gamepad2.left_stick_y);
        } else if (gamepad2.dpad_up) {
            moveElevator(1);
        } else if (gamepad2.dpad_down) {
            moveElevator(-1);
        } else if (magMove && elevatorTouch.getState()) {
            // for ele touch true means that it isn't touched
            if (currTime < goalTime) {
                moveElevator(elevatorPower);
            } else if (!magOn){
                moveElevator(elevatorPower);
            }
        } else if (!elevatorTouch.getState()) {
            magMove = false;
            elevatorPower = 0;
            leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            moveElevator(.5);
        } else {
            moveElevator(0);
        }

        // flip
        if (gamepad2.y) {
            // flip
            flip.setPosition(flip_UP);
            //flipped = !flipped;
        } else {
            flip.setPosition(flip_DOWN);
        }

        // escalator
        if (gamepad2.dpad_left) {
            escalatorPower = -1;
        } else if (gamepad2.dpad_right) {
            escalatorPower = 1;
        } else if (Math.abs(gamepad2.right_stick_y) > .25) {
            escalatorPower = Range.clip(-gamepad2.right_stick_y, -1.0, 1.0);
        } else {
            escalatorPower = 0;
        }
        escalator.setPower(escalatorPower);

        // end of code, update telemetry
        telemetry.addData("Right", gamepad1.right_stick_y);
        telemetry.addData("Left", gamepad1.left_stick_y);
        telemetry.addData("Slow", gamepad1.left_bumper);
        telemetry.addData("Elevator", leftElevator.getPower());
        telemetry.addData("Elevator Touch", elevatorTouch.getState());
        telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
        telemetry.addData("Right Encoder", -rightMotor.getCurrentPosition());
        telemetry.addData("Left Ele Encoder", leftElevator.getCurrentPosition());
        telemetry.addData("Right Ele Encoder", rightElevator.getCurrentPosition());
        telemetry.addData("Mag", !mag.getState());
        telemetry.update();
    }

    public void moveElevator(double initialSpeed) {
        int leftPos = leftElevator.getCurrentPosition();
        int rightPos = rightElevator.getCurrentPosition();

        /*
        if (leftPos - rightPos > 100) {
           leftElePow = .5*initialSpeed;
           rightElePow = initialSpeed;
        } else if (rightPos - leftPos > 100) {
            leftElePow = initialSpeed;
            rightElePow = .5*initialSpeed;
        } else {
            leftElePow = initialSpeed;
            rightElePow = initialSpeed;
        }
        */

        leftElePow = initialSpeed;
        rightElePow = initialSpeed;
        leftElevator.setPower(Range.clip(leftElePow, -1.0, 1.0));
        rightElevator.setPower(Range.clip(rightElePow, -1.0, 1.0));
    }

    @Override
    public void stop() {}
}
