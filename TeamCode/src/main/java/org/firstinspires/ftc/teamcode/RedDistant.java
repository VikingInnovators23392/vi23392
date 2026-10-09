
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/* FTC DECODE 2025 Team Viking Innovators #23392 */
/* AUTO: Red Distant - Distant position for Launch */

@Autonomous(name="RedDistant", group="Robot")
//@Disabled
public class RedDistant extends LinearOpMode {
    final double GOAL_HEADING_DEG = -22;
    final double WALL_HEADING_DEG = -90;
    final double FEED_TIME_SECONDS = 1.75; //The feeder servos run this long when a shot is requested.
    final double STOP_POWER = 0.0; //
    final double DRIVE_FULL_POWER = 1.0; //
    final double DRIVE_INTAKE_POWER = 0.5;
    final double TURN_MAX_POWER = 0.6;
    final double INTAKE_INTAKING_VELOCITY = 3600;
    final double INTAKE_LAUNCHING_VELOCITY = 2000;
    final double LAUNCHER_DISTANT_TARGET_VELOCITY = 1765; //Target velocity for distant goal
    final double LAUNCHER_DISTANT_MIN_VELOCITY = 1740; //minimum required to start a shot for distant goal.
    private enum LaunchDistantState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchDistantState launchDistantState;
    private boolean shotRequested = true;
    ElapsedTime feederTimer = new ElapsedTime();
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    viRobotHardware robot = new viRobotHardware(this);

    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init(true, true); //claw Close
        launchDistantState = LaunchDistantState.IDLE;

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.addData("Autonomous:", "Red-DISTANT");
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        robot.setLauncherVelocity(LAUNCHER_DISTANT_TARGET_VELOCITY);

        /* 1st Shoot */
        // Drive robot forward to Launch zone
        robot.drive(10, DRIVE_FULL_POWER, 0);
        // Turn the robot to CounterClockwise face GOAL
        robot.turnTo(GOAL_HEADING_DEG, TURN_MAX_POWER, 0);
        // 1st Shoot
        while(shotRequested) {
            launchDistant();
        }
        // Turn the robot to Clockwise to face WALL
        robot.turnTo(WALL_HEADING_DEG, TURN_MAX_POWER, 0);
        // Drive robot forward left to the Right side Artifacts
        robot.driveStrafe(12, 18, DRIVE_FULL_POWER, DRIVE_FULL_POWER, 0);

        /* 2nd Shoot Right Artifacts */
        // Drive the robot forward to intake the 3 Right Artifacts
        robot.drive(  32, DRIVE_INTAKE_POWER, 0);
        sleep(500);
        // Drive robot backward right to Launch zone
        robot.driveStrafe(-42, -18, DRIVE_FULL_POWER, DRIVE_FULL_POWER,0);
        // Turn the robot to CounterClockwise face GOAL
        robot.turnTo(GOAL_HEADING_DEG, TURN_MAX_POWER, 0);
        // 2nd shoot
        shotRequested = true;
        launchDistantState = LaunchDistantState.IDLE;
        while(shotRequested) {
            launchDistant();
        }
        // Turn the robot to Clockwise to face WALL
        robot.turnTo(WALL_HEADING_DEG, TURN_MAX_POWER, 0);
        // Drive robot forward left to the Middle Artifacts
        robot.driveStrafe(14, 37, DRIVE_FULL_POWER, DRIVE_FULL_POWER, 0);

        /* 3rd Shoot Left side Artifacts */
        // Driver the robot forward to intake the 3 Right side Artifacts
        robot.drive(  32, DRIVE_INTAKE_POWER, 0);
        sleep(500);
        // Drive robot backward right to Launch zone
        robot.driveStrafe(-42, -38, DRIVE_FULL_POWER, DRIVE_FULL_POWER,0);
        // Turn the robot to CounterClockwise to face GOAL
        robot.turnTo(GOAL_HEADING_DEG, TURN_MAX_POWER, 0);
        // 3rd Shoot
        shotRequested = true;
        launchDistantState = LaunchDistantState.IDLE;
        while(shotRequested) {
            launchDistant();
        }
        // Drive robot forward to leave Launch Zone
        robot.drive(10, DRIVE_FULL_POWER, 0);

        //sleep(3000);
        // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Autonomous:", "Done");
            telemetry.addData("-", "-------");
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            //sleep(50);
    }

    void launchDistant() {
        switch (launchDistantState) {
            case IDLE:
                launchDistantState = LaunchDistantState.SPIN_UP;
                break;
            case SPIN_UP:
                robot.setViperPower(0); //Intake OFF
                if (robot.getLauncherVelocity() > LAUNCHER_DISTANT_MIN_VELOCITY) {
                    launchDistantState = LaunchDistantState.LAUNCH;
                }
                break;
            case LAUNCH:
                robot.setArmPosition(0.4);  //Open feeder
                sleep(10);
                robot.setViperPower(0.5); //Intake ON
                //robot.setIntakeVelocity(INTAKE_LAUNCHING_VELOCITY); //Intake ON
                feederTimer.reset();
                launchDistantState = LaunchDistantState.LAUNCHING;
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
