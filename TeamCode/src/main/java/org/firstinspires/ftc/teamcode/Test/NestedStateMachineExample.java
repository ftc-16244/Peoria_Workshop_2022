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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="NESTED State Machine Example", group="Linear Opmode")
@Disabled
public class NestedStateMachineExample extends LinearOpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    // Declare OpMode members.
    private ElapsedTime             runtime         = new ElapsedTime();
    private ElapsedTime             lifttime        = new ElapsedTime(); // used to see how lon it takes to lift
    private ElapsedTime             teleopTimer     = new ElapsedTime();
    private DcMotor                 juan            = null;
    private DcMotor                 julio           = null;
    private static final double     TELEOP_TIME_OUT             =   122; // seconds - Automatically shuts down teleop to prevent motor damage (for practice sessions)

    // Lift (Juan) ///

    private static final double     TICKS_PER_MOTOR_REV         =   537.7; // goBilda 312 RPM motor
    private static final double     LIFT_DIST_PER_REV           =   42 * Math.PI / 25.4; // inches of lift per motor rev (42mm  winch pulley)
    public static final double      TICKS_PER_LIFT_IN           =   TICKS_PER_MOTOR_REV / LIFT_DIST_PER_REV ; // 100 and change
    private static final double     JUAN_HEIGHT_PARTIAL         =   4.0; // inches
    private static double           JUAN_SPEED_UP               =   0.9; // power
    private static double           JUAN_SPEED_DOWN             =   -0.2; // power
    private static double           JUAN_SPEED_HOLD             =   0.12;
    private static double           juanKp                      =   0.0015; //power per tick of error (Juan)
    private static double           juanKf                      =   0.25; //feed forward for the lift (Juan)

    // Arm (Julio) ///


    private static final double     JULIO_LEFT90                =   -390; // degrees
    private static final double     JULIO_RIGHT90               =  390; // degrees
    private static final double     TICKS_PER_DEGREE            =   1425.1/360; // ticks per degree
    private static final double     JULIO_SPEED_UP              =   0.5; // power to rotate up
    private static final double     JULIO_SPEED_DOWN            =   -0.15; // power to rotate back
    private static final double     JULIO_SPEED_HOLD            =   0.10; // power to hold against gravity - check signs
    private static final double     julioKp                     =  0.00112; //power per tick of error (Julio)
    private static double           julioKf                     =  0.25; //feed forward for the arm - Cosine relationship so not FINAL

   // States

    private LiftState               mliftstate; // "m" = class variable that all methods can share. Not a local variable
    private ArmState                marmstate;
    private DriverCommandState      mdrivercmdstate;

  // mover to ENUM package eventually
    private enum LiftState {
        LIFT_DOWN,
        LIFT_UP,
        LIFT_PROP_UP,
        LIFT_HOLD,
        LIFT_IDLE,
        MANUAL,
        LIFT_MECH_RESET,
        UNKNOWN
    }

    private enum ArmState {
        ARM_CENTER,
        ARM_LEFT,
        ARM_RIGHT,
        ARM_HOLD,
        ARM_PARKED,
        MANUAL,
        UNKNOWN
    }

    private enum DriverCommandState {
        RELOAD,
        ALLIANCE_HUB_LEFT,
        ALLIANCE_HUB_RIGHT,
        SHARED_HUB_LEFt,
        SHARED_HUB_RIGHT,
        UNKNOWN
    }



    @Override
    public void runOpMode() {

        int juanPosition; // local variable that only works inside "runOpmode" it is not shared to other methods
        int juanTarget = 0;
        int juanLifError;

        int julioPosition;
        int julioTarget = 0;
        int julioError;

        double juanKfCorr; // corrects feed forward for battery voltage


        String lifttimestring ="";
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        VoltageSensor  voltSensor = hardwareMap.voltageSensor.get("Expansion Hub 2"); // go to robot config, click Expansion Hub and look to see what number hub it is if you don't already know


        juan = hardwareMap.get(DcMotor.class, "juanLift");
        juan.setDirection(DcMotorEx.Direction.FORWARD);
        juan.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        juan.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        mliftstate = LiftState.UNKNOWN;


        julio = hardwareMap.get(DcMotor.class, "julioArm");
        julio.setDirection(DcMotorEx.Direction.FORWARD);
        julio.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        julio.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        marmstate = ArmState.UNKNOWN;

        mdrivercmdstate = DriverCommandState.UNKNOWN;

        juanMechanicalReset();// make sure lift is all the way down before starting

        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());

        waitForStart();
        runtime.reset();
        teleopTimer.reset(); // reset because the clock starts when you init.

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() &&  teleopTimer.seconds() < TELEOP_TIME_OUT  ) {

            juanKfCorr = (juanKf *voltSensor.getVoltage()) /12;
            double juanpower = -gamepad1.left_stick_y;
            double juliopower = gamepad1.right_stick_x;

/////////////// Game Pad 1 Assignments /////////////////////////
            // Button names and positions
            //        y
            //      x   b
            //        a


            if (gamepad1.dpad_left) {
                mdrivercmdstate = DriverCommandState.ALLIANCE_HUB_LEFT;
                mliftstate = LiftState.LIFT_UP;
                juanTarget = (int) (JUAN_HEIGHT_PARTIAL * TICKS_PER_LIFT_IN);
                julioTarget = (int) (JULIO_LEFT90 * TICKS_PER_DEGREE);
                lifttime.reset(); // reset lift time as soon as drive pushes button
            }

            if (gamepad1.dpad_right) {
                mdrivercmdstate = DriverCommandState.ALLIANCE_HUB_RIGHT;
                mliftstate = LiftState.LIFT_UP;
                juanTarget = (int) (JUAN_HEIGHT_PARTIAL * TICKS_PER_LIFT_IN);
                julioTarget = (int) (JULIO_RIGHT90 * TICKS_PER_DEGREE);
                lifttime.reset(); // reset lift time as soon as drive pushes button

            }
            if (gamepad1.dpad_down) {
                mdrivercmdstate = DriverCommandState.RELOAD;
                mliftstate = LiftState.LIFT_DOWN;
                lifttime.reset(); // reset lift time as soon as drive pushes button
                juanTarget = 0;
                julioTarget = 0;
            }
            // by moving the left joystick in either direction the driver takes manual control.
            if (Math.abs(gamepad1.left_stick_y) > 0.25) {
                mliftstate = LiftState.MANUAL;
            }

            ////////////////////////////  SWITCH CASE for states     ////////////////

            juanPosition = juan.getCurrentPosition(); // call once before switch to save methods calls.
            juanLifError = (int) (juanTarget - juanPosition); // cast as an int because target is a double

            julioPosition = julio.getCurrentPosition(); // call once before switch to save methods calls.
            julioError = (int) (julioTarget - julioPosition); // cast as an int because target is a double


            switch (mdrivercmdstate) {

                case ALLIANCE_HUB_LEFT:

                    switch (mliftstate) {

                        case LIFT_UP:
                            if (juanPosition < juanTarget) // this prevents short power up of motors if target
                            // is achieved and the driver accidentally pushes the lift up button again.
                            // without this, the motor will power on for the duration of the loop time; 10-20 ms ot so.

                            {
                                juan.setPower(juanKfCorr + (juanLifError * juanKp));
                                lifttimestring = lifttime.toString(); // convert lifttime to a string as long as case is active
                                // the timer never stops. If timer output is converted to text when we wnat, we just read the last value
                                // becase that is all we care about. It tell us how ling we were in this state. Helpful for tuning.
                            }
                            // The exit condition for LIFT_UP is reaching the target height - no operator input needed
                            // If the driver presses the DOWN button it would also change the state
                            if (juanPosition >= juanTarget) {
                                mliftstate = LiftState.LIFT_HOLD;
                                marmstate = ArmState.ARM_LEFT; //when in hold go ahead and pivot arm. this assumes the lift won't fall down too.
                                // There is no exit condition for the lift defined. THis is because we need operator input to get out of the HOLD state.
                                // In this case the ArmState.ArmLeft is an action performed in the LIFT_UP case of the lift.
                                // This is then seen as an event in the ArmState State Machine. So this is kind of an action and event depending
                                // the vantage point.
                            }
                            break;

                        case LIFT_HOLD: // Power high enough to prevent lift from dropping but not high enough to lift it.
                            juan.setPower(Math.abs(JUAN_SPEED_HOLD));

                            break;

                        case MANUAL: //Joystick control
                            // Send calculated power to lift
                            juan.setPower(juanpower);

                            // Exiting this state requires driver button press.
                            break;


                } // end of switch case Alliance Hub Left


           // break;


                case RELOAD:

                    switch (mliftstate) {

                        case LIFT_DOWN: //
                            if (marmstate != ArmState.ARM_PARKED) {
                                marmstate = ArmState.ARM_CENTER;
                            } else {
                                juan.setPower(JUAN_SPEED_DOWN);
                                lifttimestring = lifttime.toString();
                                // The exit condition for LIFT_DOWN is reaching a very low lift position - no operator input needed
                                if (juan.getCurrentPosition() < 20) {
                                    mliftstate = LiftState.LIFT_IDLE;
                                }
                            }

                            break;

                        case LIFT_IDLE: //Protects motor by turning it off as the lift is almost at the bottom.

                            juan.setPower(0);
                            // Exiting this state requires driver button press.
                            break;
                    }


                case ALLIANCE_HUB_RIGHT:

                    switch (mliftstate) {

                        case LIFT_UP:
                            if (juanPosition < juanTarget){
                                juan.setPower(juanKfCorr + (juanLifError * juanKp));
                                lifttimestring = lifttime.toString(); // convert lifttime to a string as long as case is active
                                // the timer never stops. If timer output is converted to text when we wnat, we just read the last value
                                // becase that is all we care about. It tell us how ling we were in this state. Helpful for tuning.
                            }
                            // The exit condition for LIFT_UP is reaching the target height - no operator input needed
                            // If the driver presses the DOWN button it would also change the state
                            if (juanPosition >= juanTarget) {
                                mliftstate = LiftState.LIFT_HOLD;
                                marmstate = ArmState.ARM_RIGHT; //when in hold go ahead and pivot arm. this assumes the lift won't fall down too.
                                // There is no exit condition for the lift defined. THis is because we need operator input to get out of the HOLD state.
                                // In this case the ArmState.ArmLeft is an action performed in the LIFT_UP case of the lift.
                                // This is then seen as an event in the ArmState State Machine. So this is kind of an action and event depending
                                // the vantage point.
                            }
                            break;

                        case LIFT_HOLD: // Power high enough to prevent lift from dropping but not high enough to lift it.
                            juan.setPower(Math.abs(JUAN_SPEED_HOLD));

                            break;

                        case MANUAL: //Joystick control
                            // Send calculated power to lift
                            juan.setPower(juanpower);

                            // Exiting this state requires driver button press.
                            break;


                    } // end of switch case Alliance Hub Right

                    break;

            } // end of driver switch state

            switch (marmstate){

                case ARM_LEFT:

                    if (julioPosition < julioTarget){ // only move it below te target
                        julio.setPower(-JULIO_SPEED_UP);

                    }
                    else{

                        marmstate = ArmState.ARM_HOLD;
                    }

                break;

                case ARM_RIGHT:
                    if (julioPosition < julioTarget){ // only move it below te target
                        julio.setPower(JULIO_SPEED_UP);

                    }
                    else{

                        marmstate = ArmState.ARM_HOLD;
                    }
                    break;

                case ARM_CENTER:
                    if ((julioPosition -julioTarget) > 50) {
                        julio.setPower((-0.1 + julioError * julioKp));
                    }
                    else if ((julioPosition -julioTarget) < -50){
                            julio.setPower((0.1+julioError * julioKp));
                        }

                    else{
                        marmstate = ArmState.ARM_PARKED;
                    }

                    break;

                case ARM_HOLD:

                    julio.setPower(JULIO_SPEED_HOLD + (julioError * julioKp));

                    break;

                case ARM_PARKED:

                    //julio.setPower((julioError * julioKp));
                    julio.setPower((0));

                    break;




            } // end of arm state switch case

            // Telemetry to torubleshoot switch cases
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Lift Transition Time", lifttimestring);
            telemetry.addData("Lift State is", mliftstate);
            telemetry.addData("Arm State is", marmstate);
            telemetry.addData("Lift Encoder Target is", (juanTarget));
            telemetry.addData("Lift Encoder Ticks are", juanPosition);
            telemetry.addData("Arm Encoder Target is", (julioTarget));
            telemetry.addData("Arm Encoder Ticks are", julioPosition);
            telemetry.addData("Motor Power", "Julio (%.2f)", julio.getPower());
            telemetry.addData("Battery", "  Voltage (%.2f)", voltSensor.getVoltage());
            telemetry.update();

        } // end of the while loop

    } // end of the main runopmode method

    public void juanMechanicalReset(){
        mliftstate = LiftState.MANUAL.LIFT_MECH_RESET; // we know the lift is at zero here.
        juan.setPower(JUAN_SPEED_DOWN);
        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        while ((runtime.seconds() < 2.0)) {

            telemetry.addData("Lift State is",mliftstate);
            telemetry.update();
        }
        // set everything back the way is was before reset so encoders can be used
        juan.setPower(0); // stop motor after time expires
        sleep(500);// need to pause before resetting the encoder.
        // The spring on the goBilda cascade lift allows the motor to keep turning when the lift is
        // all the way at the bottom. When power is shut off the spring turns the motor in the "lift"
        // direction. THe sleep lets the motor come to rest before encoder reset. Otherwise the ticks
        // will not be be approx zero when the lift is physically at its lowerst point.
        juan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        juan.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);// make sure to add this after reset to switch out of encoder mode
        mliftstate = LiftState.MANUAL.LIFT_IDLE;

        telemetry.addData("Lift State is",mliftstate);
        telemetry.update();

    }
}
