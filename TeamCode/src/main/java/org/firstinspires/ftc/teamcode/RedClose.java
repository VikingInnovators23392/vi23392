
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/* FTC DECODE 2025 Team Viking Innovators #23392 */
/* AUTO: Red Goal - Close position for Launch */

@Autonomous(name="RedClose", group="Robot")
//@Disabled
public class RedClose extends LinearOpMode {
    final double GOAL_HEADING_DEG = 0;
    final double WALL_HEADING_DEG = -41;
    final double FEED_TIME_SECONDS = 1.75; //The feeder servos run this long when a shot is requested.
    final double STOP_POWER = 0.0; //
    final double DRIVE_FULL_POWER = 1.0; //
    final double DRIVE_INTAKE_POWER = 0.6;
    final double TURN_MAX_POWER = 0.6;
    final double INTAKE_INTAKING_VELOCITY = 3600;
    final double INTAKE_LAUNCHING_VELOCITY = 2300;
    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1125; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1100; //minimum required to start a shot for close goal.
    private enum LaunchCloseState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchCloseState launchCloseState;
    private boolean shotRequested = true;
    ElapsedTime feederTimer = new ElapsedTime();
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    viRobotHardware robot = new viRobotHardware(this);

    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init(true, true); //claw Close
        launchCloseState = LaunchCloseState.IDLE;

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.addData("Autonomous:", "Red-CLOSE");
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        robot.setLauncherVelocity(LAUNCHER_CLOSE_TARGET_VELOCITY);

        /* 1st Shoot Initial Loaded Artifacts */
        // Drive robot backward
        robot.drive(-27, DRIVE_FULL_POWER, 0);
        // 1st Shoot
        while(shotRequested) {
            launchClose();
        }
        // Turn the robot to Clockwise to face WALL
        robot.turnTo(WALL_HEADING_DEG, TURN_MAX_POWER, 0);
        // Drive robot right to the Left side Artifacts
        robot.strafe(-24, DRIVE_FULL_POWER, 0);

        /* 2nd Shoot Left side Artifacts */
        // Driver the robot forward to intake the 3 Left side Artifacts
        robot.drive(  28, DRIVE_INTAKE_POWER, 0);  //INTAKE
        sleep(500);
        // Drive robot backward left to Launch zone
        robot.driveStrafe(-24, 22, DRIVE_FULL_POWER, DRIVE_FULL_POWER,0);
        // Turn the robot to CounterClockwise to face GOAL
        robot.turnTo(GOAL_HEADING_DEG, TURN_MAX_POWER, 0);
        // 2nd Shoot
        shotRequested = true;
        launchCloseState = LaunchCloseState.IDLE;
        while(shotRequested) {
            launchClose();
        }
        // Turn the robot to Clockwise to face WALL
        robot.turnTo(WALL_HEADING_DEG, TURN_MAX_POWER, 0);
        // Drive robot right to the Middle Artifacts
        robot.strafe(-48, DRIVE_FULL_POWER, 0);

        /* 3rd Shoot */
        // Driver the robot forward to intake the 3 Middle Artifacts
        robot.drive(  35, DRIVE_INTAKE_POWER, 0);
        sleep(500);
        // Drive robot backward to avoid GATE
        robot.drive(-15, DRIVE_FULL_POWER, 0);
        // Drive robot backward left to Launch zone
        robot.driveStrafe(-16, 46, DRIVE_FULL_POWER, DRIVE_FULL_POWER,0);
        // Turn the robot to CounterClockwise to face GOAL
        robot.turnTo(GOAL_HEADING_DEG, TURN_MAX_POWER, 0);
        // 3rd Shoot
        shotRequested = true;
        launchCloseState = LaunchCloseState.IDLE;
        while(shotRequested) {
            launchClose();
        }
        // Turn the robot to Clockwise to face the WALL
        robot.turnTo(WALL_HEADING_DEG, TURN_MAX_POWER, 0);
        // Drive robot right to the Right side Artifacts
        robot.strafe(-68, DRIVE_FULL_POWER, 0);

        // Driver the robot forward to intake the 3 Right side Artifacts
        robot.drive(  36, DRIVE_INTAKE_POWER, 0);
        robot.setIntakeVelocity(INTAKE_INTAKING_VELOCITY); //Intake ON
        sleep(500);
        robot.drive(  -20, DRIVE_FULL_POWER, 0);

        // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Autonomous:", "Done");
            telemetry.addData("-", "-------");
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            //sleep(50);
    }

    void launchClose() {
        switch (launchCloseState) {
            case IDLE:
                launchCloseState = LaunchCloseState.SPIN_UP;
                break;
            case SPIN_UP:
                robot.setViperPower(0); //Intake OFF
                if (robot.getLauncherVelocity() > LAUNCHER_CLOSE_MIN_VELOCITY) {
                    launchCloseState = LaunchCloseState.LAUNCH;
                }
                break;
            case LAUNCH:
                robot.setArmPosition(0.4);  //Open feeder
                sleep(10);
                //robot.setViperPower(0.9); //Intake ON
                robot.setIntakeVelocity(INTAKE_LAUNCHING_VELOCITY); //Intake ON
                feederTimer.reset();
                launchCloseState = LaunchCloseState.LAUNCHING;
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
