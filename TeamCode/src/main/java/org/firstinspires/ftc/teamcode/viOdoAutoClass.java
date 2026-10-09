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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/* https://github.com/gearsincorg/SimplifiedOdometry/blob/main/SampleAutonomous.java#L23 */
/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="ViOdoAutoClass", group="Robot")
@Disabled
public class viOdoAutoClass extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    viRobotHardware robot = new viRobotHardware(this);

    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init(true, false);  // Claw Close

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive()) {
            // Note, this example takes more than 30 seconds to execute, so turn OFF the auto timer.

            // Drive a large rectangle, turning at each corner

            robot.drive(  -24, 1.0, 0.0);
            //robot.stopRobot();
            //robot.drive(  24, 1.0, 0.0);

            robot.strafe( 24, 1.0, 0.0);
            //robot.stopRobot();
            //robot.strafe( -24, 1.0, 0.0);

            robot.turnTo(90, 0.6, 0.0);
            //robot.stopRobot();
            //robot.turnTo(0, 0.6, 0.0);

            //robot.driveStrafe(24, 24, 1.0,0.6,0);
            //robot.stopRobot();
            //robot.driveStrafe(-24, -24, 1.0,0.6,0);

            //robot.driveStrafeTurnto(24,24,90,1.0,0.6,1.0,0);



            robot.stopRobot();
            //sleep(500);

            // Drive the path again without turning.
/*
            robot.drive(  84, 0.60, 0.15);
            robot.strafe( 72, 0.60, 0.15);
            robot.drive( -84, 0.60, 0.15);
            robot.strafe(-72, 0.60, 0.15);
*/
            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Autonomous:", "Odometry");
            telemetry.addData("-", "-------");
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            //sleep(50);
        //}
    }
}
