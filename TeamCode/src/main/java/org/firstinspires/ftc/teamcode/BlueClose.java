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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/* FTC DECODE 2025 Team Viking Innovators #23392 */
/* AUTO: Blue Goal - Close position for Launch */

@Autonomous(name="BlueClose", group="Robot")
//@Disabled
public class BlueClose extends LinearOpMode {
    final double FEED_TIME_SECONDS = 1.75; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double INTAKE_INTAKING_VELOCITY = 3000;
    final double INTAKE_LAUNCHING_VELOCITY = 2300;
    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1150; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1125; //minimum required to start a shot for close goal.
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
        telemetry.addData("Autonomous:", "Blue-CLOSE");
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        robot.setLauncherVelocity(LAUNCHER_CLOSE_TARGET_VELOCITY);

        /* 1st Shoot */
        // Drive robot backward
        robot.drive(-35, 1.0, 0);

        // 1st Soot
        while(shotRequested) {
            launchClose();
        }
        // Turn the robot to CounterClockwise
        robot.turnTo(41, 1.0, 0);
        // Drive robot left to the Artifacts
        robot.strafe(37, 1.0, 0);

        /* 2nd Shoot */
        // Driver the robot forward to intake the 3 Artifacts
        robot.drive(  37, 0.75, 0);
        // Drive robot backward right
        robot.driveStrafe(-30, -30, 1.0, 0.8,0);
        // Turn the robot to Clockwise
        robot.turnTo(0, 1.0, 0);
        // 2nd shoot
        shotRequested = true;
        launchCloseState = LaunchCloseState.IDLE;
        while(shotRequested) {
            launchClose();
        }
        // Turn the robot to CounterClockwise
        robot.turnTo(41, 1.0, 0);
        // Drive robot left to the Artifacts
        robot.strafe(70, 1.0, 0);

        /* 3rd Shoot */
        // Driver the robot forward to intake the 3 Artifacts
        robot.drive(  45, 0.75, 0);
        // Drive robot backward to avoid GATE
        robot.drive(-15, 1.0, 0);
        // Drive robot backward right
        robot.driveStrafe(-20, -65, 1.0, 0.8,0);
        // Turn the robot to Clockwise
        robot.turnTo(0, 1.0, 0);
        // 3rd Shoot
        shotRequested = true;
        launchCloseState = LaunchCloseState.IDLE;
        while(shotRequested) {
            launchClose();
        }
        // Turn the robot to CounterClockwise
        robot.turnTo(41, 1.0, 0);
        // Drive robot left to the Artifacts
        robot.strafe(100, 1.0, 0);

        // Driver the robot forward to intake the 3 Artifacts
        robot.drive(  47, 0.75, 0);
        robot.setIntakeVelocity(INTAKE_INTAKING_VELOCITY); //Intake ON
        sleep(1000);

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
