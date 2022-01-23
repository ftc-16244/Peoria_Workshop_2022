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


@TeleOp(name="Juan State Machine Example", group="Linear Opmode")
//@Disabled
public class JuamStateMachineExample extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime teleopTimer = new ElapsedTime();
    private DcMotor juan = null;
    private DcMotor julio = null;

    private static final double     TICKS_PER_MOTOR_REV         = 537.7; // goBilda 312 RPM motor
    private static final double     LIFT_DIST_PER_REV           = 42 * Math.PI / 25.4; // inches of lift per motor rev (42mm  winch pulley)
    public static final double      TICKS_PER_LIFT_IN           = TICKS_PER_MOTOR_REV / LIFT_DIST_PER_REV ; // 100 and change

    private static final double     JUANLPARTIAL                = 5.5; // inches

    private static final double     JUANLIFTSPEED_UP            = 0.9; // power
    private static final double     JUANLIFTSPEED_DOWN          = -0.2; // power
    private static final double     JUANLIFTSPEED_HOLD          = 0.2;
    private static final double     maxTeleopTimeOut            = 200; // seconds to automotically shut down teleop after to protect motors

    private enum LiftState {
        LIFT_DOWN,
        LIFT_UP,
        LIFT_HOLD,
        LIFT_IDLE,
        MANUAL
    }

    private enum ArmState {
        ARM_CENTER,
        ARM_LEFT,
        ARM_RIGHT
    }

    private LiftState mliftstate; // m = member of the class as opposed to a local variable inside the "runOpmoode" method
    private ArmState marmstate;

    @Override
    public void runOpMode() {

        int juanPosition; // local variable that onlt works inside "runOpmode"

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        juan = hardwareMap.get(DcMotor.class, "juanLift");
        juan.setDirection(DcMotorEx.Direction.FORWARD);
        juan.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        juan.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        julio = hardwareMap.get(DcMotor.class, "julioArm");
        julio.setDirection(DcMotorEx.Direction.FORWARD);
        julio.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        julio.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() &&  teleopTimer.seconds() < maxTeleopTimeOut) {

            double juanpower = -gamepad1.left_stick_y;
            double juliopower = gamepad1.right_stick_x;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "juan (%.2f), julio (%.2f)", juanpower, juliopower);
            telemetry.update();

///////////////Driver initiated state transition via the game pad//////////////////////////

            //        y
            //      x   b
            //        a


            if (gamepad1.y) {
                mliftstate = LiftState.LIFT_UP;
            }
            if (gamepad1.a) {
                mliftstate = LiftState.LIFT_DOWN;
            }

            if (Math.abs(gamepad1.left_stick_y) > 0.25) {
                mliftstate = LiftState.MANUAL;
            }

            ////////////////////////////  SWITCH CASE for states     ////////////////

            juanPosition = juan.getCurrentPosition(); // call once before switch to save methods calls

            switch (mliftstate){

                case LIFT_UP:

                    if  (juanPosition < JUANLPARTIAL  *  TICKS_PER_LIFT_IN ) // this prevents short power up of motore each time button is pressed.
                    {juan.setPower(Math.abs(JUANLIFTSPEED_UP));
                    }
                    // The exit condition for LIFT_UP is reaching the target height - no operator input needed
                    // If the driver presses the DOWN button it would also change the state
                    if( juanPosition >= JUANLPARTIAL  *  TICKS_PER_LIFT_IN  ){
                        mliftstate = LiftState.LIFT_HOLD;
                    }

                    break;

                case LIFT_DOWN: //
                    juan.setPower(JUANLIFTSPEED_DOWN);
                    // The exit condition for LIFT_DOWN is reaching a very low lift position - no operator input needed
                    if(juan.getCurrentPosition() < 20){
                        mliftstate = LiftState.LIFT_IDLE;
                    }

                    break;

                case LIFT_HOLD: // Power high enough to prevent lift from dropping but not high enough to lift it.
                    juan.setPower(Math.abs(JUANLIFTSPEED_HOLD));
                    // There is no exit condition defined in this case. Exisitng this state requires the driver to press up or down.
                    // Pressing up will just put the state back in hold f the lift is up. You would need to press DOWN.
                    break;

                case LIFT_IDLE: //Protects motor by turning it off as the lift is almost at the bottom.

                    juan.setPower(0);
                    // Exiting this state requires driver button press.
                    break;


                case MANUAL: //Joystick control
                    // Send calculated power to wheels
                    juan.setPower(juanpower);

                    // Exiting this state requires driver button press.
                    break;



            }
            // Telemetry to torubleshoot switch cases
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lift State is",mliftstate);
            telemetry.addData("Lift Encoder Ticks are",juanPosition);
            telemetry.addData("Motor Power is", juan.getPower());
            telemetry.update();




/**
            if(mliftstate == LiftState.LIFT_UP){
                telemetry.addData("Lift State is","UP");
                telemetry.addData("Target Position is", JUANLPARTIAL  *  TICKS_PER_LIFT_IN);
                if (juanPosition < JUANLPARTIAL  *  TICKS_PER_LIFT_IN ) // this prevents indexing up
                    {juan.setPower(Math.abs(JUANLIFTSPEED_UP));
                    telemetry.addData("Lift Counts",juanPosition);
                    telemetry.update();
                }
                if( juanPosition >= JUANLPARTIAL  *  TICKS_PER_LIFT_IN  ){
                    mliftstate = LiftState.LIFT_HOLD;

                }

            }

            if(mliftstate == LiftState.LIFT_HOLD){
                telemetry.addData("Lift State is","HOLD");
                telemetry.addData("Lift Counts",juanPosition);
                telemetry.update();
                juan.setPower(Math.abs(JUANLIFTSPEED_HOLD));

            }

            if(mliftstate == LiftState.LIFT_DOWN){
                telemetry.addData("Lift State is","DOWN");
                telemetry.addData("Lift Counts",juanPosition);
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
*/
        }
    }
}
