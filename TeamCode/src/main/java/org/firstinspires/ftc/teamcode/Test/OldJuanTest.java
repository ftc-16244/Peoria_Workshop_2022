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

package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="Old Juan Test", group="Linear Opmode")
//@Disabled
public class OldJuanTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor juan = null;
    private DcMotor julio = null;

    private static final double     TICKS_PER_MOTOR_REV         = 537.7; // goBilda 312 RPM motor
    private static final double     LIFT_DIST_PER_REV           = 42*3.14159/25.4; // inches of lift per motor rev
    public static final double      TICKS_PER_LIFT_IN           = TICKS_PER_MOTOR_REV / LIFT_DIST_PER_REV ; // 100 and change

    private static final double     JUANLPARTIAL                = 8; // inches

    private static final double     JUANLLOAD                   = 0;

    private static final double     JUANLIFTSPEED_UP            = 0.9; // power
    private static final double     JUANLIFTSPEED_DOWN          = -0.1; // power
    private static final double     JUANLIFTSPEED_HOLD          = 0.2;

    private enum LiftState {
        LIFT_DOWN,
        LIFT_UP,
        LIFT_HOLD,
        LIFT_IDLE
    }

    private enum ArmState {
        ARM_CENTER,
        ARM_LEFT,
        ARM_RIGHT
    }

    private LiftState mliftstate;
    private ArmState marmstate;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        juan = hardwareMap.get(DcMotor.class, "juanLift");
        //juan.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        juan.setDirection(DcMotorEx.Direction.FORWARD);
        juan.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        juan.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        julio = hardwareMap.get(DcMotor.class, "julioArm");
        //julio.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        julio.setDirection(DcMotorEx.Direction.FORWARD);
        julio.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        julio.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double juanpower = -gamepad1.left_stick_y;
            double juliopower = gamepad1.right_stick_x;

            // Send calculated power to wheels
            juan.setPower(juanpower);
            julio.setPower(juliopower);

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "juan (%.2f), julio (%.2f)", juanpower, juliopower);
            telemetry.update();
///////////////Driver sets the state transistions with the gamepad///////////////////////////
            if (gamepad1.y) {
                mliftstate = LiftState.LIFT_UP;
                //telemetry.addData("Lift State is","UP");

            }
            if (gamepad1.a) {
                mliftstate = LiftState.LIFT_DOWN;
                //telemetry.addData("Lift State is","DOWN");
            }

            //////////////////////////////Actions we look for for the State that is set////////////////

            if(mliftstate == LiftState.LIFT_UP){
                telemetry.addData("Lift State is","UP");
                telemetry.addData("Target Position is", JUANLPARTIAL  *  TICKS_PER_LIFT_IN);
                if (juan.getCurrentPosition() < JUANLPARTIAL  *  TICKS_PER_LIFT_IN ) // this prevents indexing up
                    {juan.setPower(Math.abs(JUANLIFTSPEED_UP));
                    telemetry.addData("Lift Counts",juan.getCurrentPosition());
                    telemetry.update();
                }
                if( juan.getCurrentPosition() >= JUANLPARTIAL  *  TICKS_PER_LIFT_IN  ){
                    mliftstate = LiftState.LIFT_HOLD;

                }

            }

            if(mliftstate == LiftState.LIFT_HOLD){
                telemetry.addData("Lift State is","HOLD");
                telemetry.addData("Lift Counts",juan.getCurrentPosition());
                telemetry.update();
                juan.setPower(Math.abs(JUANLIFTSPEED_HOLD));

            }

            if(mliftstate == LiftState.LIFT_DOWN){
                telemetry.addData("Lift State is","DOWN");
                telemetry.addData("Lift Counts",juan.getCurrentPosition());
                telemetry.addData("Motor Down Power is", juan.getPower());
                telemetry.update();

                juan.setPower(JUANLIFTSPEED_DOWN);
                if(juan.getCurrentPosition() < 20){
                    mliftstate = LiftState.LIFT_IDLE;
                }

            }

            if(mliftstate == LiftState.LIFT_IDLE){
                telemetry.addData("Lift State is","IDLE");
                telemetry.addData("Lift Counts",juan.getCurrentPosition());
                telemetry.update();
                juan.setPower(0);
            }

            telemetry.update();
        }
    }
}
