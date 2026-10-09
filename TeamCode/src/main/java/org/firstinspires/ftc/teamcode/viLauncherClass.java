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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="viLauncherClass", group="Robot")
//@Disabled
public class viLauncherClass extends LinearOpMode {
    final double FEEDER_CLOSE_POSITION = 0.75;
    final double FEEDER_OPEN_POSITION = 0.4;
    final double FEED_TIME_SECONDS = 2.0; //The feeder servos run this long when a shot is requested.
    final double FEED_TIME_SECONDS_DISTANT = 3.0;
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.

    final double INTAKE_INTAKING_VELOCITY = 3600;
    final double INTAKE_LAUNCHING_VELOCITY = 2000;
    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1100; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1075; //minimum required to start a shot for close goal.
    final double LAUNCHER_FAR_TARGET_VELOCITY = 1400; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1375; //minimum required to start a shot for far goal.
    final double LAUNCHER_DISTANT_TARGET_VELOCITY = 1765; //Target velocity for far goal
    final double LAUNCHER_DISTANT_MIN_VELOCITY = 1740; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_DISTANT_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_DISTANT_MIN_VELOCITY;
    //boolean shotingRequested = false;
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private enum LaunchCloseState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private enum LaunchFarState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private enum LaunchDistantState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;
    private LaunchCloseState launchCloseState;
    private LaunchFarState launchFarState;
    private LaunchDistantState launchDistantState;
    private enum IntakeState {
        ON,
        OFF;
    }
    private IntakeState intakeState = IntakeState.OFF;
    private enum LauncherDistance {
        CLOSE,
        FAR;
    }
    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;
    ElapsedTime feederTimer = new ElapsedTime();
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    viRobotHardware robot = new viRobotHardware(this);

    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init(false, false);

        launchState = LaunchState.IDLE;
        launchCloseState = LaunchCloseState.IDLE;
        launchFarState = LaunchFarState.IDLE;
        launchDistantState = LaunchDistantState.IDLE;
        //feederTimer.reset();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //robot.driveOmni(0.8);
            robot.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            /*
             * Here we give the user control of the speed of the launcher motor without automatically
             * queuing a shot.
             */
            if (gamepad2.y) {
                robot.setLauncherVelocity(launcherTarget);
            }
            else if (gamepad2.b) {  // stop flywheel
                robot.setLauncherVelocity(STOP_SPEED);
            }

            if (gamepad2.aWasPressed()){
                switch (intakeState){
                    case ON:
                        intakeState = IntakeState.OFF;
                        robot.setViperPower(0);
                        break;
                    case OFF:
                        intakeState = IntakeState.ON;
                        //robot.setViperPower(1);
                        robot.setIntakeVelocity(INTAKE_INTAKING_VELOCITY);
                        break;
                }
            }

            if (gamepad2.dpadUpWasPressed()) {
                switch (launcherDistance) {
                    case CLOSE:
                        launcherDistance = LauncherDistance.FAR;
                        launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                        launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
                        break;
                    case FAR:
                        launcherDistance = LauncherDistance.CLOSE;
                        launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                        launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
                        break;
                }
            }

            launchClose(gamepad2.leftBumperWasPressed());
            launchFar(gamepad2.rightBumperWasPressed());
            launchDistant(gamepad2.xWasPressed());

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Ascent Up/Down", "gamepad2 Y/A");
            telemetry.addLine("dpad | ")
                    .addData("Up", gamepad2.dpad_up)
                    .addData("Down", gamepad2.dpad_down);
            //telemetry.addData("intakeState ", intakeState);
            telemetry.addData("launcherTarget ", "%.2f", launcherTarget);
            telemetry.addData("feederTimer ", "%5.1f", feederTimer.seconds());
            telemetry.addData("IntakeVelocity ", robot.getIntakeVelocity());
            telemetry.addData("launcherVelocity ", robot.getLauncherVelocity());
            //telemetry.addData("State", launchState);
            telemetry.addData("CloseState", launchCloseState);
            telemetry.addData("FarState", launchFarState);
            telemetry.update();


            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        } //while (opModeIsActive())
    }

    void launchClose(boolean shotRequested) {
        switch (launchCloseState) {
            case IDLE:
                if (shotRequested) {
                    launchCloseState = LaunchCloseState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                intakeState = IntakeState.OFF;
                robot.setViperPower(0); //Intake OFF
                robot.setLauncherVelocity(LAUNCHER_CLOSE_TARGET_VELOCITY);
                if (robot.getLauncherVelocity() > LAUNCHER_CLOSE_MIN_VELOCITY) {
                    launchCloseState = LaunchCloseState.LAUNCH;
                }
                break;
            case LAUNCH:
                robot.setArmPosition(FEEDER_OPEN_POSITION);  //Open feeder
                sleep(500);
                intakeState = IntakeState.ON;
                robot.setViperPower(0.6); //Intake ON
                feederTimer.reset();
                launchCloseState = LaunchCloseState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    robot.setArmPosition(FEEDER_CLOSE_POSITION);  //Close feeder
                    intakeState = IntakeState.ON;
                    //robot.setViperPower(1); //Intake ON
                    robot.setIntakeVelocity(INTAKE_INTAKING_VELOCITY); //Intake ON
                    robot.setLauncherVelocity(STOP_SPEED);
                    launchCloseState = LaunchCloseState.IDLE;
                }
                break;
        }
    }

    void launchFar(boolean shotRequested) {
        switch (launchFarState) {
            case IDLE:
                if (shotRequested) {
                    launchFarState = LaunchFarState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                intakeState = IntakeState.OFF;
                robot.setViperPower(0); //Intake OFF
                robot.setLauncherVelocity(LAUNCHER_FAR_TARGET_VELOCITY);
                if (robot.getLauncherVelocity() > LAUNCHER_FAR_MIN_VELOCITY) {
                    launchFarState = LaunchFarState.LAUNCH;
                }
                break;
            case LAUNCH:
                robot.setArmPosition(FEEDER_OPEN_POSITION);  //Open feeder
                sleep(500);
                intakeState = IntakeState.ON;
                robot.setViperPower(0.6); //Intake ON
                feederTimer.reset();
                launchFarState = LaunchFarState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    robot.setArmPosition(FEEDER_CLOSE_POSITION);  //Close feeder
                    intakeState = IntakeState.ON;
                    //robot.setViperPower(1); //Intake ON
                    robot.setIntakeVelocity(INTAKE_INTAKING_VELOCITY); //Intake ON
                    robot.setLauncherVelocity(STOP_SPEED);
                    launchFarState = LaunchFarState.IDLE;
                }
                break;
        }
    }
    void launchDistant(boolean shotRequested) {
        switch (launchDistantState) {
            case IDLE:
                if (shotRequested) {
                    launchDistantState = LaunchDistantState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                intakeState = IntakeState.OFF;
                robot.setViperPower(0); //Intake OFF
                robot.setLauncherVelocity(LAUNCHER_DISTANT_TARGET_VELOCITY);
                if (robot.getLauncherVelocity() > LAUNCHER_DISTANT_MIN_VELOCITY) {
                    launchDistantState = LaunchDistantState.LAUNCH;
                }
                break;
            case LAUNCH:
                robot.setArmPosition(FEEDER_OPEN_POSITION);  //Open feeder
                sleep(500);
                intakeState = IntakeState.ON;
                robot.setViperPower(0.5); //Intake ON
                feederTimer.reset();
                launchDistantState = LaunchDistantState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS_DISTANT) {
                    robot.setArmPosition(FEEDER_CLOSE_POSITION);  //Close feeder
                    intakeState = IntakeState.ON;
                    //robot.setViperPower(1); //Intake ON
                    robot.setIntakeVelocity(INTAKE_INTAKING_VELOCITY); //Intake ON
                    robot.setLauncherVelocity(STOP_SPEED);
                    launchDistantState = LaunchDistantState.IDLE;
                }
                break;
        }
    }
    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                robot.setLauncherVelocity(launcherTarget);
                if (robot.getLauncherVelocity() > launcherMin) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                robot.setArmPosition(FEEDER_OPEN_POSITION);  //Open feeder
                sleep(500);
                intakeState = IntakeState.ON;
                robot.setViperPower(0.5);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    robot.setArmPosition(FEEDER_CLOSE_POSITION);  //Close feeder
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
}
