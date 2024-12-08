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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot2024;

/**
 * This is the TeleOp Mode control class for 2021 robot.
 */

@TeleOp(name = "ElevatorTest", group = "Iterative Opmode")
@Disabled
public class ElevatorTest extends OpMode {
    /*
     * Code to run ONCE when the driver hits INIT
     */
    private final ElapsedTime loopTime = new ElapsedTime();

    //Vertical Servo control params
    private final double vertRotateServoTimerResetVal = 0;
    private final double vertClawServoTimerResetVal = 0;
    private final double horzServoCmdTimeResetVal = 0;
    private final double grabberServoCmdTimerResetVal = 0;
    private final double vertReceiveWaitTimerResetVal = 0;
    private final double clawRotateDownTimerResetVal = 0;
    private final double clawRetractTimerResetVal = 0;
    private final double slowModeTimerResetVal = 0;

    private double dt_s=.1;

    Robot2024 robot;

    //Teleop variables


    double prvCmd = 0;
    @Override

    public void init() {
        robot = new Robot2024(this);
        robot.resetImplements();
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY.
     */
    @Override
    public void start() {
        loopTime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        //Vertical Elevator Control
        double elevatorCommand = -gamepad2.left_stick_y*1;

//        double desiredVelocity = elevatorCommand * 400;
//        robot.updateElevatorVelocityControl(desiredVelocity,dt_s);
        robot.raiseLowerElevator(elevatorCommand);
//        robot.elevatorUpdate(loopTime.seconds());

        ///To only update the postion when intending to (up and co0mmand is greater and down and commnd is going down
//        if (prvCmd < elevatorCommand && elevatorCommand >= 0.05){
//            robot.setDesElevatorPosition_Teliop(robot.getElevatorPositition());
//            robot.raiseLowerElevator(elevatorCommand);
//        }else if (prvCmd > elevatorCommand && elevatorCommand <= -0.05){
//            robot.setDesElevatorPosition_Teliop(robot.getElevatorPositition());
//            robot.raiseLowerElevator(elevatorCommand);
//        }
//        prvCmd = elevatorCommand;
//        ///////

//        RobotLog.i(String.format("elapsed Time %.3f",loopTime.seconds()));

//        telemetry.addData("#ofTicks", robot.getTicks());
//        telemetry.update();
        while(loopTime.milliseconds() < 100){
            RobotLog.i("looptime is: %.3f ",loopTime.milliseconds());
        }
        RobotLog.i("loopTimeReset: %.3f",loopTime.milliseconds());
        dt_s = loopTime.seconds();
        loopTime.reset();


    }
}
