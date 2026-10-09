/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class viRobotHardware {
    // Adjust these numbers to suit your robot.
    private final double ODOM_INCHES_PER_COUNT   = 0.002969;   //  GoBilda Odometry Pod (1/226.8)
    private final boolean INVERT_DRIVE_ODOMETRY  = false;      //  When driving FORWARD, the odometry value MUST increase.  If it does not, flip the value of this constant.
    private final boolean INVERT_STRAFE_ODOMETRY = false;      //  When strafing to the LEFT, the odometry value MUST increase.  If it does not, flip the value of this constant.

    private static final double DRIVE_GAIN          = 0.03;    //was 0.03 // Strength of axial position control
    private static final double DRIVE_ACCEL         = 0.5;     //was 2.0 // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE     = 1.0;     //was 0.5 // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND      = 0.2;     //was 0.2 // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double DRIVE_MAX_AUTO      = 0.6;     //was 0.6 // "default" Maximum Axial power limit during autonomous

    private static final double STRAFE_GAIN         = 0.03;    //was 0.03 // Strength of lateral position control
    private static final double STRAFE_ACCEL        = 0.75;     //was 1.5 // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double STRAFE_TOLERANCE    = 2.5;     //was 0.5 // Controller is is "inPosition" if position error is < +/- this amount
    private static final double STRAFE_DEADBAND     = 0.2;     //was 0.2// Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double STRAFE_MAX_AUTO     = 0.6;     //was 0.6 // "default" Maximum Lateral power limit during autonomous

    private static final double YAW_GAIN            = 0.018;   //was 0.018 // Strength of Yaw position control
    private static final double YAW_ACCEL           = 1.5;     //was 3.0 // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     //was 1.0 // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    //was 0.25 // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     //was 0.6 // "default" Maximum Yaw power limit during autonomous

    // Public Members
    public double driveDistance     = 0; // scaled axial distance (+ = forward)
    public double strafeDistance    = 0; // scaled lateral distance (+ = left)
    public double heading           = 0; // Latest Robot heading from IMU

    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    public ProportionalControl driveController     = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public ProportionalControl strafeController    = new ProportionalControl(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    public ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    private ElapsedTime holdTimer = new ElapsedTime();

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    // Declare OpMode members for each of the 4 motors.
    private IMU imu;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor driveEncoder;       //  the Axial (front/back) Odometry Module (may overlap with motor, or may not)
    private DcMotor strafeEncoder;      //  the Lateral (left/right) Odometry Module (may overlap with motor, or may not)
    private DcMotorEx viperMotor = null;
    private DcMotorEx ascentMotor = null;
    private Servo   armServo = null;
    private Servo   clawServo = null;
    private Servo   ascendServo = null;
    private Servo   xServo = null;
    private Servo   yServo = null;
    private Servo   gripperServo = null;
    private Servo   extendServo = null;


    private int rawDriveOdometer    = 0; // Unmodified axial odometer count
    private int driveOdometerOffset = 0; // Used to offset axial odometer
    private int rawStrafeOdometer   = 0; // Unmodified lateral odometer count
    private int strafeOdometerOffset= 0; // Used to offset lateral odometer
    private double rawHeading       = 0; // Unmodified heading (degrees)
    private double headingOffset    = 0; // Used to offset heading

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU
    private boolean showTelemetry     = false;

    //Arm
    private static final double ARM_UP = 0.75;
    // Claw
    private static final double CLAW_OPEN = 0.65;
    private static final double CLAW_CLOSE = 0.75;
    // Gripper
    private static final double X_SERVO = 0;
    private static final double Y_SERVO = 0;
    private static final double GRIPPER_OPEN = 0.45;
    private static final double GRIPPER_CLOSE = 1;
    private static final double EXTEND_SERVO = 0.52;
    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double VIPER_POWER =  1.0 ;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public viRobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(boolean showTelemetry, boolean clawPosition)    {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        // IMU
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // goBILDA 5203 Series
        leftBackDrive  = setupDriveMotor( "left_back_drive", DcMotor.Direction.REVERSE);   // Controller Hub Motor Port 0
        rightBackDrive = setupDriveMotor( "right_back_drive",DcMotor.Direction.FORWARD);   // Controller Hub Motor Port 1
        leftFrontDrive  = setupDriveMotor("left_front_drive", DcMotor.Direction.REVERSE);  // Controller Hub Motor Port 2
        rightFrontDrive = setupDriveMotor("right_front_drive", DcMotor.Direction.FORWARD); // Controller Hub Motor Port 3
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // goBILDA 5203 Series
        viperMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "viper");                     // Expansion Hub Port 0
        ascentMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "ascent");                 // Expansion Hub Port 1

        //  Connect to the encoder channels using the name of that channel.
        // 2000 Countable Events per Revolution
        driveEncoder = myOpMode.hardwareMap.get(DcMotor.class, "axial");    // front/back   // Expansion Hub Port 2
        strafeEncoder = myOpMode.hardwareMap.get(DcMotor.class, "lateral"); // left/right   // Expansion Hub Port 3

        // REV Smart Robot Servo
        armServo  = myOpMode.hardwareMap.get(Servo.class, "arm");                          // Controller Hub Servo Port 0
        clawServo = myOpMode.hardwareMap.get(Servo.class, "claw");                         // Controller Hub Servo Port 1
        ascendServo = myOpMode.hardwareMap.get(Servo.class, "ascend");                     // Controller Hub Servo Port 2
        // Gripper Servo

        xServo = myOpMode.hardwareMap.get(Servo.class, "x_servo");                         // Controller Hub Servo Port 3
        yServo = myOpMode.hardwareMap.get(Servo.class, "y_servo");                         // Controller Hub Servo Port 4
        gripperServo = myOpMode.hardwareMap.get(Servo.class, "gripper");                   // Controller Hub Servo Port 5
        extendServo = myOpMode.hardwareMap.get(Servo.class, "extend");                     // Expansion Hub  Servo Port 0

        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Tell the software how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        //RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        //RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
                        //RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        //RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // zero out all the odometry readings.
        resetOdometry();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;

        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //viperMotor.setDirection(DcMotor.Direction.REVERSE);

        ascentMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ascentMotor.setDirection(DcMotor.Direction.REVERSE);
        //ascentMotor.setPower(0);
        ascentMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        armServo.setPosition(ARM_UP);  // Arm Up
        if (clawPosition) {
            clawServo.setPosition(CLAW_CLOSE); // Claw Close
        } else {
            clawServo.setPosition(CLAW_OPEN); // Claw Open
        }
        ascendServo.setPosition(0);  // Ascend (Servo) Down

        gripperServo.setPosition(GRIPPER_CLOSE); // gripper Close
        myOpMode.sleep(1000);
        yServo.setPosition(Y_SERVO);
        myOpMode.sleep(1000);
        xServo.setPosition(X_SERVO);
        myOpMode.sleep(1000);
        extendServo.setPosition(EXTEND_SERVO);

        myOpMode.telemetry.addData("TeleOp", "Driver-Controlled");
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        // Send telemetry message to indicate successful Encoder reset
        //myOpMode.telemetry.addData("Starting at",  "%7d", getArmPosition());
        myOpMode.telemetry.update();
    } //public void init()

    public void imuInit(double xRotation, double yRotation, double zRotation) {
        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        // Now initialize the IMU with this mounting orientation
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    public void imuResetYaw() {
        imu.resetYaw();
    }
    public YawPitchRollAngles  imuGetRobotYawPitchRollAngles() {
        return imu.getRobotYawPitchRollAngles();
    }
    public AngularVelocity imuGetRobotAngularVelocity(AngleUnit angleUnit) {
        return imu.getRobotAngularVelocity(angleUnit);
    }

    public void driveOmni(double wheelPower) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            // Wheels Control
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -myOpMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  myOpMode.gamepad1.left_stick_x;
            double yaw     =  myOpMode.gamepad1.right_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * wheelPower);
            rightFrontDrive.setPower(rightFrontPower * wheelPower);
            leftBackDrive.setPower(leftBackPower * wheelPower);
            rightBackDrive.setPower(rightBackPower * wheelPower);
        }
    }

    public void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }
    public void moveRobotForward(double power) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            // Wheels Control
            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(power), Math.abs(power));
            max = Math.max(max, Math.abs(power));
            max = Math.max(max, Math.abs(power));
            if (max > 1.0) {
                power /= max;
            }
            // Send powers to the wheels.
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);
        }
    }
    public void moveRobotBackward(double power) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            // Wheels Control
            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(power), Math.abs(power));
            max = Math.max(max, Math.abs(power));
            max = Math.max(max, Math.abs(power));
            if (max > 1.0) {
                power /= max;
            }
            // Send powers to the wheels.
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);
        }
    }
    public void moveRobotRight(double power) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            // Wheels Control
            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(power), Math.abs(power));
            max = Math.max(max, Math.abs(power));
            max = Math.max(max, Math.abs(power));
            if (max > 1.0) {
                power /= max;
            }
            // Send powers to the wheels.
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
        }
    }
    public void moveRobotLeft(double power) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            // Wheels Control
            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(power), Math.abs(power));
            max = Math.max(max, Math.abs(power));
            max = Math.max(max, Math.abs(power));
            if (max > 1.0) {
                power /= max;
            }
            // Send powers to the wheels.
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
        }
    }
    public void turnRobotClockwise(double power) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            // Wheels Control
            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(power), Math.abs(power));
            max = Math.max(max, Math.abs(power));
            max = Math.max(max, Math.abs(power));
            if (max > 1.0) {
                power /= max;
            }
            // Send powers to the wheels.
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
        }
    }
    public void turnRobotCounterClockwise(double power) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            // Wheels Control
            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(power), Math.abs(power));
            max = Math.max(max, Math.abs(power));
            max = Math.max(max, Math.abs(power));
            if (max > 1.0) {
                power /= max;
            }
            // Send powers to the wheels.
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
        }
    }

    public void resetViperEncoder() {
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double getViperPower() {
        return viperMotor.getPower();
    }
    public double getIntakeVelocity() {
        return viperMotor.getVelocity();
    }
    public void setIntakeVelocity(double velocity) {
        viperMotor.setVelocity(velocity);
    }
    public void setViperPower(double power) {
        viperMotor.setPower(power);
    }

    public int getViperPosition() {
        return viperMotor.getCurrentPosition();
    }

    public void setViperPosition(int targetViperPosition) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            viperMotor.setTargetPosition(targetViperPosition);
            // Turn On RUN_TO_POSITION
            viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion.
            viperMotor.setPower(VIPER_POWER);

            /*
            while (myOpMode.opModeIsActive() && viperMotor.isBusy()) {
                // Display it for the driver.
                myOpMode.telemetry.addLine("left joystick | ")
                        .addData("x", myOpMode.gamepad1.left_stick_x)
                        .addData("y", myOpMode.gamepad1.left_stick_y);
                myOpMode.telemetry.addData("Viper Target Position ", "%7d", (int) targetViperPosition);
                myOpMode.telemetry.addData("Viper Current Position", "%7d", getViperPosition());
                myOpMode.telemetry.update();
            }
            */
        }

    }

    public void resetAscentEncoder() {
        ascentMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double getAscentPower() {
        return ascentMotor.getPower();
    }
    public void setAscentPower(double power) {
        ascentMotor.setPower(power);
    }
    public double getLauncherVelocity() {
        return ascentMotor.getVelocity();
    }
    public void setLauncherVelocity(double velocity) {
        ascentMotor.setVelocity(velocity);
    }
    public int getAscentPosition() {
        return ascentMotor.getCurrentPosition();
    }
    public void setAscentPosition(int targetAscentPosition) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            ascentMotor.setTargetPosition(targetAscentPosition);
            // Turn On RUN_TO_POSITION
            ascentMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion.
            ascentMotor.setPower(1.0);
        }
    }

    public void setArmPosition(double arm_position) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            armServo.setPosition(arm_position);
        }
    }
    public double getArmPosition() {
        return armServo.getPosition();
    }

    public void setClawPosition(double claw_position) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            clawServo.setPosition(claw_position);
        }
    }
    public double getClawPosition() {
        return clawServo.getPosition();
    }

    public void setAscendPosition(double ascend_position) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            ascendServo.setPosition(ascend_position);
        }
    }
    public double getAscendPosition() {
        return ascendServo.getPosition();
    }

    public void setXPosition(double x_position) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            xServo.setPosition(x_position);
        }
    }
    public void setYPosition(double y_position) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            yServo.setPosition(y_position);
        }
    }
    public void setGripperPosition(double gripper_position) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            gripperServo.setPosition(gripper_position);
        }
    }
    public void setExtendPosition(double extend_position) {
        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            extendServo.setPosition(extend_position);
        }
    }


    // Computes the current battery voltage
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : myOpMode.hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    /**
     *   Setup a drive motor with passed parameters.  Ensure encoder is reset.
     * @param deviceName  Text name associated with motor in Robot Configuration
     * @param direction   Desired direction to make the wheel run FORWARD with positive power input
     * @return the DcMotor object
     */
    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    /**
     * Read all input devices to determine the robot's motion
     * always return true so this can be used in "while" loop conditions
     * @return true
     */
    public boolean readSensors() {
        rawDriveOdometer = driveEncoder.getCurrentPosition() * (INVERT_DRIVE_ODOMETRY ? -1 : 1);
        rawStrafeOdometer = strafeEncoder.getCurrentPosition() * (INVERT_STRAFE_ODOMETRY ? -1 : 1);
        driveDistance = (rawDriveOdometer - driveOdometerOffset) * ODOM_INCHES_PER_COUNT;
        strafeDistance = (rawStrafeOdometer - strafeOdometerOffset) * ODOM_INCHES_PER_COUNT;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        rawHeading  = orientation.getYaw(AngleUnit.DEGREES);
        heading     = rawHeading - headingOffset;
        turnRate    = angularVelocity.zRotationRate;
/*
        if (showTelemetry) {
            myOpMode.telemetry.addData("Odom Ax:Lat", "%6d %6d", rawDriveOdometer - driveOdometerOffset, rawStrafeOdometer - strafeOdometerOffset);
            myOpMode.telemetry.addData("Dist Ax:Lat", "%5.2f %5.2f", driveDistance, strafeDistance);
            myOpMode.telemetry.addData("Head Deg:Rate", "%5.2f %5.2f", heading, turnRate);
        }
 */
        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void drive(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(distanceInches, power);   // achieve desired drive distance
        strafeController.reset(0);              // Maintain zero strafe drift
        yawController.reset();                          // Maintain last turn heading
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param distanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void strafe(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(0.0);             //  Maintain zero drive drift
        strafeController.reset(distanceInches, power);  // Achieve desired Strafe distance
        yawController.reset();                          // Maintain last turn angle
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {

        yawController.reset(headingDeg, power);
        while (myOpMode.opModeIsActive() && readSensors()) {

            // implement desired axis powers
            moveRobot(0, 0, yawController.getOutput(heading));

            // Time to exit?
            if (yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    public void driveStrafe(double driveInches, double strafeInches,
                                double drivePower, double strafePower,
                                double holdTime) {
        resetOdometry();

        driveController.reset(driveInches, drivePower);    // Achieve desired Drive distance
        strafeController.reset(strafeInches, strafePower); // Achieve desired Strafe distance
        yawController.reset();
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    public void driveStrafeTurnto(double driveInches, double strafeInches, double headingDeg,
                                  double drivePower, double strafePower, double turnPower,
                                  double holdTime) {
        resetOdometry();
        yawController.reset();

        driveController.reset(driveInches, drivePower);    // Achieve desired Drive distance
        strafeController.reset(strafeInches, strafePower); // Achieve desired Strafe distance
        yawController.reset(headingDeg, turnPower);
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    //  ########################  Low level control functions.  ###############################

    /**
     * Drive the wheel motors to obtain the requested axes motions
     * @param drive     Fwd/Rev axis power
     * @param strafe    Left/Right axis power
     * @param yaw       Yaw axis power
     */
    public void moveRobot(double drive, double strafe, double yaw){

        double lF = drive - strafe - yaw;
        double rF = drive + strafe + yaw;
        double lB = drive + strafe - yaw;
        double rB = drive - strafe + yaw;

        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));

        //normalize the motor values
        if (max > 1.0)  {
            lF /= max;
            rF /= max;
            lB /= max;
            rB /= max;
        }

        //send power to the motors
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);
/*
        if (showTelemetry) {
            myOpMode.telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
            myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
            myOpMode.telemetry.update(); //  Assume this is the last thing done in the loop.
        }
 */
        if (showTelemetry) {
            myOpMode.telemetry.addData("Viper Current Position", "%7d", getViperPosition());
            myOpMode.telemetry.addData("Viper Current Power", "%.2f", getViperPower());
            myOpMode.telemetry.update(); //  Assume this is the last thing done in the loop.
        }
    }

    /**
     * Stop all motors.
     */
    public void stopRobot() {
        moveRobot(0,0,0);
    }

    /**
     * Set odometry counts and distances to zero.
     */
    public void resetOdometry() {
        readSensors();
        driveOdometerOffset = rawDriveOdometer;
        driveDistance = 0.0;
        driveController.reset(0);

        strafeOdometerOffset = rawStrafeOdometer;
        strafeDistance = 0.0;
        strafeController.reset(0);
    }

    /**
     * Reset the robot heading to zero degrees, and also lock that heading into heading controller.
     */
    public void resetHeading() {
        readSensors();
        headingOffset = rawHeading;
        yawController.reset(0);
        heading = 0;
    }

    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    /**
     * Set the drive telemetry on or off
     */
    public void showTelemetry(boolean show){
        showTelemetry = show;
    }
}

//****************************************************************************************************
//****************************************************************************************************

/***
 * This class is used to implement a proportional controller which can calculate the desired output power
 * to get an axis to the desired setpoint value.
 * It also implements an acceleration limit, and a max power output.
 */
class ProportionalControl {
    double  lastOutput;
    double  gain;
    double  accelLimit;
    double  defaultOutputLimit;
    double  liveOutputLimit;
    double  setPoint;
    double  tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    ElapsedTime cycleTime = new ElapsedTime();

    public ProportionalControl(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.defaultOutputLimit = outputLimit;
        this.liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        reset(0.0);
    }

    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input  Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double input) {
        double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;
        double output;

        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }

        inPosition = (Math.abs(error) < tolerance);

        // Prevent any very slow motor output accumulation
        if (Math.abs(error) <= deadband) {
            output = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            output = (error * gain);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((output - lastOutput) > dV) {
                output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
                output = lastOutput - dV;
            }
        }

        lastOutput = output;
        cycleTime.reset();
        return output;
    }

    public boolean inPosition(){
        return inPosition;
    }
    public double getSetpoint() {return setPoint;}

    /**
     * Saves a new setpoint and resets the output power history.
     * This call allows a temporary power limit to be set to override the default.
     * @param setPoint
     * @param powerLimit
     */
    public void reset(double setPoint, double powerLimit) {
        liveOutputLimit = Math.abs(powerLimit);
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * @param setPoint
     */
    public void reset(double setPoint) {
        liveOutputLimit = defaultOutputLimit;
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Leave everything else the same, Just restart the acceleration timer and set output to 0
     */
    public void reset() {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
    }
}
