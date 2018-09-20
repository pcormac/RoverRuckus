/*
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

import android.app.Activity;
import android.graphics.Color;
import android.hardware.Sensor;
import android.util.Log;
import android.view.View;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import java.text.DecimalFormat;

@Disabled
@Autonomous(name="AutoFunctions", group="Main")
public class AutoFunctions extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */

    // Device declarations
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor elevator = null;
    DcMotor escalator = null;
    Servo servoGrabberLeft = null;
    Servo servoGrabberRight = null;
    Servo servoJewel = null;

    //TouchSensor escalatorTouch = null;
    DigitalChannel elevatorTouch = null;
    OpticalDistanceSensor odsSensor;
    OpticalDistanceSensor sharpIR = null;
    ColorSensor colorSensor = null;
    //ColorSensor colorSensorCloser = null; // closer to the robot
    ModernRoboticsI2cRangeSensor ultra = null;

    // Variables to be used for later
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    public static final String VUFORIA_KEY = "AezNI5P/////AAAAGau6WS2ao0KEr5u8BAb3J9lTKExRB45YfNKMCkXuhNSkvqjxenFuSAeL148drmLgkGK+HFayX4nihmiiQ49o3HHT5j/TBwjrVpktUmGQyOjREYq78kQAS1CuQMYa6PKSTLAID+lK/RvJxZ4pqH6H4LgxQBsKaKqzhMr96hwAE2uKKdf1j3gzOyRl0Gsrqv63QiXfJCeJtM1GcuQziKlWv1y6+BcKa+hi31FxvggkMEHVxh7xQ4yAHUvmxQHINOndIE6C6Ltz+ef+7KhCkwmk/QVTPDUafTSuJMPTSp5RxNIK+L+U4kjEMZOFhYP2ubczRypICjcg93tpP73+xvPHyyDIeSoieWvkDdJXhmcKIhrA"; // Insert your own key here

    public float robotX = 0;
    public float robotY = 0;
    public float robotAngle = 0;

    double jewelUp = .95;
    double jewelDown = .38;


    String color;
    String vMark;

    boolean vuforiaFound = false;

    public void runOpMode() throws InterruptedException {
        // get motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        elevator = hardwareMap.dcMotor.get("elevator");
        escalator = hardwareMap.dcMotor.get("escalator");

        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // get servos
        servoGrabberLeft = hardwareMap.servo.get("servoGrabberLeft");
        servoGrabberRight = hardwareMap.servo.get("servoGrabberRight");
        servoJewel = hardwareMap.servo.get("servoJewel");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        //odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        ultra = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");
        //escalatorTouch = hardwareMap.touchSensor.get("escalatorTouch");
        elevatorTouch = hardwareMap.get(DigitalChannel.class, "elevatorTouch");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        elevatorTouch.setMode(DigitalChannel.Mode.INPUT);

        colorSensor.enableLed(true);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        waitForStart();

        idle();
    }
    public void updateColor() throws InterruptedException {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
    }
    public void openGrabbers() throws InterruptedException {
        servoGrabberLeft.setPosition(.45);
        servoGrabberRight.setPosition(.50);
    }
    public void closeGrabbers() throws InterruptedException {
        servoGrabberLeft.setPosition(.65);
        servoGrabberRight.setPosition(.30);
    }
    public void getColor() throws InterruptedException {
        boolean found = false;

        sleep(1000);

        for (int i = 1; i <= 2; i++) {
            if ((colorSensor.red() > colorSensor.blue()) && (colorSensor.red() > colorSensor.alpha())) {
                color = "Red";
                found = true;
                telemetry.addData("AutoStatus", "found " + color + " on try " + i);
                telemetry.update();
                sleep(3000);
                i = 3;
            }
            else if (((colorSensor.blue() > colorSensor.red()) && (colorSensor.blue() > colorSensor.alpha()))) {
                color = "Blue";
                found = true;
                telemetry.addData("AutoStatus", "found " + color + " on try " + i);
                telemetry.update();
                sleep(3000);
                i = 3;
            }
            else {
                sleep(1000);
            }
        }

        for (int i = 3; i <= 4; i++) {
            if (!found) {
                turnSlowUsingEncoders("left", 5);
                if ((colorSensor.red() > colorSensor.blue()) && (colorSensor.red() > colorSensor.alpha())) {
                    color = "Red";
                    found = true;
                    telemetry.addData("AutoStatus", "found " + color + " on try " + i);
                    telemetry.update();
                    sleep(3000);
                } else if (((colorSensor.blue() > colorSensor.red()) && (colorSensor.blue() > colorSensor.alpha()))) {
                    color = "Blue";
                    found = true;
                    telemetry.addData("AutoStatus", "found " + color + " on try " + i);
                    telemetry.update();
                    sleep(3000);
                } else {
                    sleep(1000);
                }
            }
        }
        if (!found) {
            color = "none";
            telemetry.addData("AutoStatus", "abandoning color");
            telemetry.update();
            sleep(3000);
        }
    }
    public void hitColor(String teamColor) throws InterruptedException {
        if (teamColor.equals("Blue") || teamColor.equals("blue")) {
            if (color.equals("Red")) {
                turnSlowUsingEncoders("left", 50);
                servoJewel.setPosition(jewelUp);
                sleep(500);
                turnSlowUsingEncoders("right", 50);
            } else if (color.equals("Blue")) {
                turnSlowUsingEncoders("right", 50);
                servoJewel.setPosition(jewelUp);
                sleep(500);
                turnSlowUsingEncoders("left", 50);
            } else {
                servoJewel.setPosition(jewelUp);
            }
        } else if (teamColor.equals("Red") || teamColor.equals("red")) {
            if (color.equals("Red")) {
                turnSlowUsingEncoders("right", 50);
                servoJewel.setPosition(jewelUp);
                sleep(500);
                turnSlowUsingEncoders("left", 50);
            } else if (color.equals("Blue")) {
                turnSlowUsingEncoders("left", 50);
                servoJewel.setPosition(jewelUp);
                sleep(500);
                turnSlowUsingEncoders("right", 50);
            } else {
                servoJewel.setPosition(jewelUp);
            }
        }
    }
    public void elevatorDown() throws InterruptedException {
        closeGrabbers();
        sleep(100);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && !isStopRequested() && elevatorTouch.getState()) {
            elevator.setPower(-1);
            idle();
        }
        closeGrabbers();
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void moveElevator(double power, long timeToRun) throws InterruptedException {
        elevator.setPower(power);
        sleep(timeToRun);
        elevator.setPower(0);
        idle();
    }
    public void moveElevatorUsingEncoder(double speed, long ticks) throws InterruptedException {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (((Math.abs(elevator.getCurrentPosition())) < (Math.abs(ticks))) && !(isStopRequested()) && (opModeIsActive())) {
            elevator.setPower(speed);
            telemetry.addData("Elevator: ", elevator.getCurrentPosition());
            telemetry.addData("Goal: ", ticks);
            telemetry.update();
            idle();
        }
        elevator.setPower(0);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runForTime(double leftSpeed, double rightSpeed, long timeToRun) throws InterruptedException {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
        sleep(timeToRun);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        idle();
    }
    public void runUsingEncoders (double  leftSpeed, double rightSpeed, long distanceToRun) throws InterruptedException {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (((Math.abs(leftMotor.getCurrentPosition()) < (Math.abs(distanceToRun))) && (Math.abs(rightMotor.getCurrentPosition())) < (Math.abs(distanceToRun))) && (opModeIsActive())) {
            leftMotor.setPower(leftSpeed);
            rightMotor.setPower(rightSpeed);
            telemetry.addData("Left: ", leftMotor.getCurrentPosition());
            telemetry.addData("Right: ", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncoders (long distanceToRun) throws InterruptedException {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (((Math.abs(leftMotor.getCurrentPosition()) < (Math.abs(distanceToRun))) && (Math.abs(rightMotor.getCurrentPosition())) < (Math.abs(distanceToRun))) && (opModeIsActive())) {
            leftMotor.setPower(.25);
            rightMotor.setPower(.25);
            telemetry.addData("Left: ", leftMotor.getCurrentPosition());
            telemetry.addData("Right: ", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void turnUsingEncoders (String direction, long distanceToTurn) {
        telemetry.addData("AutoStatus: ", "Start of turnUsingEncoders");
        telemetry.update();
        sleep(500);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double speed = .5;

        if (direction.equals("left") || direction.equals("Left")) {
            while (((Math.abs(leftMotor.getCurrentPosition()) < (Math.abs(distanceToTurn))) && (Math.abs(rightMotor.getCurrentPosition())) < (Math.abs(distanceToTurn))) && (opModeIsActive())) {
                leftMotor.setPower(-speed);
                rightMotor.setPower(speed);
                telemetry.addData("AutoStatus: ", "turning using encoders");
                telemetry.addData("Left: ", leftMotor.getCurrentPosition());
                telemetry.addData("Right: ", -rightMotor.getCurrentPosition());
                telemetry.addData("Direction: ", direction);
                telemetry.addData("Goal: ", distanceToTurn);
                telemetry.update();
                idle();
            }
        } else if (direction.equals("right") || direction.equals("Right")) {
            while (((Math.abs(leftMotor.getCurrentPosition()) < (Math.abs(distanceToTurn))) && (Math.abs(rightMotor.getCurrentPosition())) < (Math.abs(distanceToTurn))) && (opModeIsActive())) {
                leftMotor.setPower(speed);
                rightMotor.setPower(-speed);
                telemetry.addData("AutoStatus: ", "turning using encoders");
                telemetry.addData("Left: ", leftMotor.getCurrentPosition());
                telemetry.addData("Right: ", -rightMotor.getCurrentPosition());
                telemetry.addData("Direction: ", direction);
                telemetry.addData("Goal: ", distanceToTurn);
                telemetry.update();
                idle();
            }
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void turnSlowUsingEncoders (String direction, long distanceToTurn) {
        telemetry.addData("AutoStatus: ", "Start of turnUsingEncoders");
        telemetry.update();
        sleep(500);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double speed = .125;

        if (direction.equals("left") || direction.equals("Left")) {
            while (((Math.abs(leftMotor.getCurrentPosition()) < (Math.abs(distanceToTurn))) && (Math.abs(rightMotor.getCurrentPosition())) < (Math.abs(distanceToTurn))) && (opModeIsActive())) {
                leftMotor.setPower(-speed);
                rightMotor.setPower(speed);
                telemetry.addData("AutoStatus: ", "turning using encoders");
                telemetry.addData("Left: ", leftMotor.getCurrentPosition());
                telemetry.addData("Right: ", -rightMotor.getCurrentPosition());
                telemetry.addData("Direction: ", direction);
                telemetry.addData("Goal: ", distanceToTurn);
                telemetry.update();
                idle();
            }
        } else if (direction.equals("right") || direction.equals("Right")) {
            while (((Math.abs(leftMotor.getCurrentPosition()) < (Math.abs(distanceToTurn))) && (Math.abs(rightMotor.getCurrentPosition())) < (Math.abs(distanceToTurn))) && (opModeIsActive())) {
                leftMotor.setPower(speed);
                rightMotor.setPower(-speed);
                telemetry.addData("AutoStatus: ", "turning using encoders");
                telemetry.addData("Left: ", leftMotor.getCurrentPosition());
                telemetry.addData("Right: ", -rightMotor.getCurrentPosition());
                telemetry.addData("Direction: ", direction);
                telemetry.addData("Goal: ", distanceToTurn);
                telemetry.update();
                idle();
            }
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingUltra (double distance, double maxTime) {
        runtime.reset();
        while ((ultra.cmUltrasonic() > distance) && opModeIsActive() && !isStopRequested() && (runtime.seconds() < maxTime)) {
            leftMotor.setPower(.25);
            rightMotor.setPower(.25);
        }
    }
    public void checkVuforia() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        runtime.reset();
        while (runtime.seconds() < 5.0 && opModeIsActive() && !isStopRequested()) {
            relicTrackables.activate();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            RelicRecoveryVuMark KEY;
            KEY = vuMark;
            if (KEY == RelicRecoveryVuMark.LEFT) {
                vMark = "Left";
                telemetry.addData("VuMark: ", "Left");
                telemetry.update();
                sleep(2000);
                break;
            } else if (KEY == RelicRecoveryVuMark.CENTER) {
                vMark = "Center";
                telemetry.addData("VuMark: ", "Center");
                telemetry.update();
                sleep(2000);
                break;
            } else if (KEY == RelicRecoveryVuMark.RIGHT) {
                vMark = "Right";
                telemetry.addData("VuMark: ", "Right");
                telemetry.update();
                sleep(2000);
                break;
            } else {
                vMark = "None";
                telemetry.addData("VuMark: ", "Not Visible");
                telemetry.update();
                sleep(50);
            }
            idle();
        }
    }
    public void checkVuforiaForTime(double time) throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        runtime.reset();
        while (runtime.seconds() < time && opModeIsActive() && !isStopRequested() && !vuforiaFound) {
            relicTrackables.activate();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            RelicRecoveryVuMark KEY;
            KEY = vuMark;
            if (KEY == RelicRecoveryVuMark.LEFT) {
                vMark = "Left";
                telemetry.addData("VuMark", "Left");
                telemetry.update();
                vuforiaFound = true;
                sleep(2000);
                break;
            } else if (KEY == RelicRecoveryVuMark.CENTER) {
                vMark = "Center";
                telemetry.addData("VuMark", "Center");
                telemetry.update();
                vuforiaFound = true;
                sleep(2000);
                break;
            } else if (KEY == RelicRecoveryVuMark.RIGHT) {
                vMark = "Right";
                telemetry.addData("VuMark", "Right");
                telemetry.update();
                vuforiaFound = true;
                sleep(2000);
                break;
            } else {
                vMark = "None";
                telemetry.addData("VuMark: ", "Not Visible");
                telemetry.update();
                sleep(50);
            }
            idle();
        }
    }
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    public void declareDevices() throws InterruptedException {
        // Device declarations
        DcMotor leftMotor = null;
        DcMotor rightMotor = null;
        DcMotor elevator = null;
        DcMotor escalator = null;
        Servo servoGrabberLeft = null;
        Servo servoGrabberRight = null;
        Servo servoJewel = null;

        //TouchSensor escalatorTouch = null;
        DigitalChannel elevatorTouch = null;
        //OpticalDistanceSensor odsSensor;
        ColorSensor colorSensor = null;
        //OpticalDistanceSensor sharpIR = null;
        //ModernRoboticsI2cRangeSensor ultra = null;

        double odsMax;
        double leftTurn;
        double rightTurn;
        boolean odsTurn;
        boolean odsTurning;
        String color;
        String vMark;
        boolean vuforiaFound = false;
        double jewelUp = .8;
        double jewelDown = .4;

        // Variables to be used for later
        final String TAG = "Vuforia VuMark Sample";
        OpenGLMatrix lastLocation = null;
        VuforiaLocalizer vuforia;
        final String VUFORIA_KEY = "AezNI5P/////AAAAGau6WS2ao0KEr5u8BAb3J9lTKExRB45YfNKMCkXuhNSkvqjxenFuSAeL148drmLgkGK+HFayX4nihmiiQ49o3HHT5j/TBwjrVpktUmGQyOjREYq78kQAS1CuQMYa6PKSTLAID+lK/RvJxZ4pqH6H4LgxQBsKaKqzhMr96hwAE2uKKdf1j3gzOyRl0Gsrqv63QiXfJCeJtM1GcuQziKlWv1y6+BcKa+hi31FxvggkMEHVxh7xQ4yAHUvmxQHINOndIE6C6Ltz+ef+7KhCkwmk/QVTPDUafTSuJMPTSp5RxNIK+L+U4kjEMZOFhYP2ubczRypICjcg93tpP73+xvPHyyDIeSoieWvkDdJXhmcKIhrA"; // Insert your own key here

        float robotX = 0;
        float robotY = 0;
        float robotAngle = 0;

    }
    public void declareMap() throws InterruptedException {
        //      Drive motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //      Elevator motor
        elevator = hardwareMap.dcMotor.get("elevator");
        //      Escalator motor
        escalator = hardwareMap.dcMotor.get("escalator");

        //      Servo
        servoGrabberLeft = hardwareMap.servo.get("servoGrabberLeft");
        servoGrabberRight = hardwareMap.servo.get("servoGrabberRight");
        servoJewel = hardwareMap.servo.get("servoJewel");

        // Sensors
        //     Light sensor
        //odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        //     Ultrasonic Sensor
        //ultra = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");
        //     Touch sensor
        //escalatorTouch = hardwareMap.touchSensor.get("touchSensor");
        //     Elevator touch
        elevatorTouch = hardwareMap.get(DigitalChannel.class, "elevatorTouch");
        //     Color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        //     Ir Sensor
        //sharpIR = hardwareMap.opticalDistanceSensor.get("infrared");


        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);


    }
}