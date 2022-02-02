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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Experiment - Iterative Opmode", group="Iterative Opmode")
@Disabled
public class Experiment_Iterative_Opmode extends OpMode
{

   //THis does not help multi tasking dirvetrain and implement.

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    // Declare OpMode members.

    private ElapsedTime             lifttime        = new ElapsedTime(); // used to see how lon it takes to lift
    private ElapsedTime             teleopTimer     = new ElapsedTime();
    private DcMotor                 juanLift            = null;
    private DcMotor                 julioArm          = null;
    private Servo homieBox         = null;
    private static final double     TELEOP_TIME_OUT             =   122; // seconds - Automatically shuts down teleop to prevent motor damage (for practice sessions)

    // Lift (Juan) ///

    private static final double     TICKS_PER_MOTOR_REV         =   537.7; // goBilda 312 RPM motor
    private static final double     LIFT_DIST_PER_REV           =   42 * Math.PI / 25.4; // inches of lift per motor rev (42mm  winch pulley)
    public static final double      TICKS_PER_LIFT_IN           =   TICKS_PER_MOTOR_REV / LIFT_DIST_PER_REV ; // 100 and change
    private static final double     JUAN_HEIGHT_PARTIAL         =   4.0; // inches
    private static double           JUAN_SPEED_UP               =   0.9; // power
    private static double           JUAN_SPEED_DOWN             =   -0.2; // power
    private static double           JUAN_SPEED_HOLD             =   0.10;
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

    public static final double      HOMIEBOXPIVOTLEFT       = 1;
    public static final double      HOMIEBOXPIVOTRIGHT      = 0.3;
    public static final double      HOMIEBOXPIVOTCENTER     = 0.66;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "Left_front");
        rightDrive = hardwareMap.get(DcMotor.class, "Right_front");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        juanLift = hardwareMap.get(DcMotor.class, "juanLift");
        juanLift.setDirection(DcMotorEx.Direction.FORWARD);
        juanLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        juanLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        julioArm = hardwareMap.get(DcMotor.class, "julioArm");
        julioArm.setDirection(DcMotorEx.Direction.FORWARD);
        julioArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        julioArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        homieBox = hardwareMap.get(Servo.class,"homieBox");

        homieBox.setPosition(HOMIEBOXPIVOTCENTER);

        juanMechanicalReset();// make sure lift is all the way down before starting

        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());





        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;



        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        if (gamepad1.dpad_left) {
           liftToTargetHeight(6,2);
           rotateToTargetAngle(JULIO_LEFT90, 3);

        }

        if (gamepad1.dpad_right) {
            liftToTargetHeight(6,2);
            rotateToTargetAngle(JULIO_RIGHT90, 3);

        }
        if (gamepad1.dpad_down) {
            rotateToTargetAngle(0, 3);
            liftToTargetHeight(0,2);


        }





    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void liftToTargetHeight(double height, double timeoutS){

        int newTargetHeight;


        // Ensure that the opmode is still active


            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetHeight = (int)(height *  TICKS_PER_LIFT_IN);
            // Set the target now that is has been calculated
            juanLift.setTargetPosition(newTargetHeight); //1000 ticks extends lift from 295mm to 530 mm which is 9.25 in per 1000 ticks or 108 ticks per in
            // Turn On RUN_TO_POSITION
            juanLift.setPower(Math.abs(JUAN_SPEED_UP));
            // reset the timeout time and start motion.
            runtime.reset();
            juanLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and there is time left, and thr motor is running.
            // Note: We use (isBusy() in the loop test, which means that when the motor hits
            // its target position, motion will stop.

            while (runtime.seconds() < timeoutS && (juanLift.isBusy())) {

                // Display it for the driver.
                //  telemetry.addData("Moving to New Lift Height",  "Running to %7d", newTargetHeight);

                // telemetry.update();
            }

            // Stop all motion after exiting the while loop
            juanLift.setPower(.25); // puts a low power to help hold the lift in place. There is a better way
            //liftPosition = LiftPosition.HOLD;
            // Turn off RUN_TO_POSITION
            //felipe.juanLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void rotateToTargetAngle(double angle, double timeoutS){

        int newTargetAngle; // ok to leave an an int unless we want really fine tuning


        // Ensure that the opmode is still active


            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetAngle = (int)(angle *  TICKS_PER_DEGREE);
            // Set the target now that is has been calculated
            julioArm.setTargetPosition(newTargetAngle); //
            // Turn On RUN_TO_POSITION
            julioArm.setPower(Math.abs(  JULIO_SPEED_UP  ));
            // reset the timeout time and start motion.
            runtime.reset();
            julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and there is time left, and thr motor is running.
            // Note: We use (isBusy() in the loop test, which means that when the motor hits
            // its target position, motion will stop.

            while ((runtime.seconds() < timeoutS) && julioArm.isBusy()) {

                // Display it for the driver.
                //  telemetry.addData("Moving to New Lift Height",  "Running to %7d", newTargetHeight);

                // telemetry.update();
            }


    }

    public void juanMechanicalReset(){
        juanLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);// make sure to add this after reset to switch out of encoder mode
        juanLift.setPower(JUAN_SPEED_DOWN);
        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        while ((runtime.seconds() < 2.0)) {


            telemetry.update();
        }
        // set everything back the way is was before reset so encoders can be used
        juanLift.setPower(0); // stop motor after time expires

        // The spring on the goBilda cascade lift allows the motor to keep turning when the lift is
        // all the way at the bottom. When power is shut off the spring turns the motor in the "lift"
        // direction. THe sleep lets the motor come to rest before encoder reset. Otherwise the ticks
        // will not be be approx zero when the lift is physically at its lowerst point.
        juanLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        juanLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        telemetry.update();

    }


}
