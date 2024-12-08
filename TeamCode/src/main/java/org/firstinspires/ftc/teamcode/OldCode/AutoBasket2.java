/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.OldCode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot2024;
import org.firstinspires.ftc.teamcode.utilities.CASH_Drive_Library;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoRedBasket2", group="Autonomous LinearOpMode")
@Disabled
public class AutoBasket2 extends LinearOpMode {
    private Robot2024 robot;
    //This sensor is used to detect the team prop.  There are two of them, one on left and one on
    //right.  The each sensor is used for a different start location of the robot depending on
    //color and alliance.
    private DistanceSensor sensorRange;
    //Variable to hold the distance value measured from the Distance sensor
    private double distance;

    //Variable that holds the runtime of the operation
    private ElapsedTime looptime = new ElapsedTime();

    //Toggle between running with just the driving of the robot if we are using second test bed to practice.
    //Set this to true to run with full robot
    //Set this to false if you want to run just the driving portion of the bot.
    private boolean initimpliments = true;
    public CASH_Drive_Library CASHDriveLibrary;

    final private double distanceToCage = 22;
    final private double distanceBackToWall = 20;
    final private double distanceToParking = 50;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        //create the robot object that basically activates everything in Robot2024.java file.
        robot = new Robot2024(this);

        robot.initializeRobot();
        robot.resetIMU();
//        robot.initializeImplements();
        CASHDriveLibrary = robot.CASHDriveLibrary;
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            robot.rotateRobotAuto2(robot.TURN_RIGHT, 90, 0.5);
            robot.updateRobotRotation(looptime.seconds());



            telemetry.addData("Done ", robot.getTicks());
            telemetry.update();
//            sleep(30000);
            looptime.reset();
         }

   }

   //This method is what is used to get the average from the desired sensor.
   double getAverageDistanceFromSensor(DistanceSensor dist_sensor){
       double NumberOfSamples=0;
       double Sum=0;
       double Average;
       double dist;
       while ( NumberOfSamples <= 100  && opModeIsActive() ) {
           dist = dist_sensor.getDistance(DistanceUnit.INCH);
           Sum = Sum + dist;
           NumberOfSamples = NumberOfSamples + 1;
           telemetry.addData("distance: ", dist);
           telemetry.update();
       }
       Average = Sum / NumberOfSamples;
       telemetry.addData("Average: ", Average);
       telemetry.update();
       return Average;
   }
}