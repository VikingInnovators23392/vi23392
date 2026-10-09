
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/* FTC DECODE 2025 Team Viking Innovators #23392 */
/* AUTO: Blue Goal - Far position for Launch */

@Autonomous(name="BlueFar", group="Robot")
//@Disabled
public class BlueFar extends LinearOpMode {
    final double GOAL_HEADING_DEG = 43;
    final double WALL_HEADING_DEG = 90;
    final double FEED_TIME_SECONDS = 1.75; //The feeder servos run this long when a shot is requested.
    final double STOP_POWER = 0.0; //
    final double DRIVE_FULL_POWER = 1.0; //
    final double DRIVE_INTAKE_POWER = 0.6;
    final double TURN_MAX_POWER = 0.6;
    final double INTAKE_INTAKING_VELOCITY = 3600;
    final double INTAKE_LAUNCHING_VELOCITY = 2000;
    final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.
    private enum LaunchFarState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchFarState launchFarState;
    private boolean shotRequested = true;
    ElapsedTime feederTimer = new ElapsedTime();
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    viRobotHardware robot = new viRobotHardware(this);

    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init(true, true); //claw Close
        launchFarState = LaunchFarState.IDLE;

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.addData("Autonomous:", "Blue-FAR");
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        robot.setLauncherVelocity(LAUNCHER_FAR_TARGET_VELOCITY);

        /* 1st Shoot */
        // Drive robot forward to Launch zone
        robot.drive(70, DRIVE_FULL_POWER, 0);
        // Turn the robot to CounterClockwise to face GOAL
        robot.turnTo(GOAL_HEADING_DEG, TURN_MAX_POWER, 0);
        // 1st Shoot
        while(shotRequested) {
            launchFar();
        }
        // Turn the robot to CounterClockwise to face WALL
        robot.turnTo(WALL_HEADING_DEG, TURN_MAX_POWER, 0);
        // Drive robot forward left to the Middle Artifacts
        robot.driveStrafe(14, 22, DRIVE_FULL_POWER, DRIVE_FULL_POWER, 0);

        /* 2nd Shoot Middle Artifacts */
        // Drive the robot forward to intake the 3 Middle Artifacts
        robot.drive(  35, DRIVE_INTAKE_POWER, 0);
        sleep(500);
        // Drive robot backward to avoid GATE
        robot.drive(-15, DRIVE_FULL_POWER, 0);
        // Drive robot backward right to Launch zone
        robot.driveStrafe(-30, -22, DRIVE_FULL_POWER, DRIVE_FULL_POWER,0);
        // Turn the robot to Clockwise face GOAL
        robot.turnTo(GOAL_HEADING_DEG, TURN_MAX_POWER, 0);
        // 2nd shoot
        shotRequested = true;
        launchFarState = LaunchFarState.IDLE;
        while(shotRequested) {
            launchFar();
        }
        // Turn the robot to CounterClockwise to face WALL
        robot.turnTo(WALL_HEADING_DEG, TURN_MAX_POWER, 0);
        // Drive robot forward left to the Left side Artifacts
        robot.driveStrafe(14, 42, DRIVE_FULL_POWER, DRIVE_FULL_POWER, 0);

        /* 3rd Shoot Left side Artifacts */
        // Driver the robot forward to intake the 3 Left side Artifacts
        robot.drive(  34, DRIVE_INTAKE_POWER, 0);
        sleep(500);
        // Drive robot backward right to Launch zone
        robot.driveStrafe(-46, -42, DRIVE_FULL_POWER, DRIVE_FULL_POWER,0);

        // Turn the robot to Clockwise to face GOAL
        robot.turnTo(GOAL_HEADING_DEG, TURN_MAX_POWER, 0);
        // 3rd Shoot
        shotRequested = true;
        launchFarState = LaunchFarState.IDLE;
        while(shotRequested) {
            launchFar();
        }

        // Drive robot left to leave Launch Zone
        robot.strafe(20, DRIVE_FULL_POWER, 0);

        //sleep(3000);
        // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Autonomous:", "Done");
            telemetry.addData("-", "-------");
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            //sleep(50);
    }

    void launchFar() {
        switch (launchFarState) {
            case IDLE:
                launchFarState = LaunchFarState.SPIN_UP;
                break;
            case SPIN_UP:
                robot.setViperPower(0); //Intake OFF
                if (robot.getLauncherVelocity() > LAUNCHER_FAR_MIN_VELOCITY) {
                    launchFarState = LaunchFarState.LAUNCH;
                }
                break;
            case LAUNCH:
                robot.setArmPosition(0.4);  //Open feeder
                sleep(10);
                //robot.setViperPower(0.9); //Intake ON
                robot.setIntakeVelocity(INTAKE_LAUNCHING_VELOCITY); //Intake ON
                feederTimer.reset();
                launchFarState = LaunchFarState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    robot.setArmPosition(0.75);  //Close feeder
                    //robot.setViperPower(1); //Intake ON
                    robot.setIntakeVelocity(INTAKE_INTAKING_VELOCITY); //Intake ON
                    shotRequested = false;
                }
                break;
        }
    }
}
