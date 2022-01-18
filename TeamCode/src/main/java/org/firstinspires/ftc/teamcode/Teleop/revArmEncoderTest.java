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

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Encoder;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Rev Encoder Test", group="Linear Opmode")
//@Disabled
public class revArmEncoderTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm = null;
    private DcMotor armEncoder = null;

    private static final double     TICKS_PER_REV   =   8192; // REV Through Bore Encoder
    private static final double     TICKS_PER_DEG  =    TICKS_PER_REV/360; // REV Through Bore Encoder
    private static final double     ARM_DEG_LEFT_TARGET_A =    90; // Degrees for huan reading

    @Override
    public void runOpMode() {

        double  armTarget = 0;
        double  armDegrees = 0;
        double  armError = 1000; // initially set to ~45 degrees to make sure error is not ero at start
        double  armPower;
        double  armKp = 0.00035; // initial guess at gain. 0.7 power with 90 degree error.
        double  timeout = 3; //seconds

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Configure the arm motor here - without encoders
        arm  = hardwareMap.get(DcMotor.class, "julioArm");
        arm.setDirection(DcMotor.Direction.FORWARD );
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       // Configure the REV Through ore Encoder here

        armEncoder  = hardwareMap.get(DcMotor.class, "julioArm");
        armEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // switch to encoder mode to reset
        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset
        armEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // switch bach to just reading the encoder

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // move the arm 90 degrees with left dpad button
            if (gamepad2.dpad_left){
                armTarget =  ARM_DEG_LEFT_TARGET_A * TICKS_PER_DEG;
                arm.setPower(.3);
                runtime.reset();
                while (runtime.time() < timeout && -armEncoder.getCurrentPosition() <= armTarget){
                    armError = armTarget -armEncoder.getCurrentPosition();
                    armPower =  armError * armKp;
                    arm.setPower( armPower); // power decreases with error
                    armDegrees= -armEncoder.getCurrentPosition()/ TICKS_PER_DEG;
                    telemetry.addData("Arm Rotation in Degrees", armDegrees);
                    telemetry.update();
                }
                arm.setPower(0);
            }

        }
    }

    private void manualEncoderReset(){
        armEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // switch to encoder mode to reset
        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset
        armEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // switch bach to just reading the encoder
    }
}
