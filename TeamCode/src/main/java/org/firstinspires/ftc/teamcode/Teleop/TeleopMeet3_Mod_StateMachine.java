package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.JulioPosition;
import org.firstinspires.ftc.teamcode.Enums.LiftPosition;
import org.firstinspires.ftc.teamcode.Enums.LiftState;
import org.firstinspires.ftc.teamcode.Enums.PatrickState;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.Felipe4;
import org.firstinspires.ftc.teamcode.Subsystems.FelipeTrois;
import org.firstinspires.ftc.teamcode.Test.BetterStateMachine;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name="Meet 3 Teleop Modified FSM",group = "Test")
public class TeleopMeet3_Mod_StateMachine extends LinearOpMode {

    private Felipe4                 felipe                      = new Felipe4(this);
    private CarouselTurnerThingy    carousel                    = new CarouselTurnerThingy();
    private ElapsedTime             runtime                     = new ElapsedTime();
    private ElapsedTime             lifttime                    = new ElapsedTime(); // used to see how long it takes to lift
    private ElapsedTime             armtime                     = new ElapsedTime(); // used to see how long it takes the arm to pivot
    private ElapsedTime             PIDtimer                    = new ElapsedTime(); // used to see how long it takes the arm to pivot

    private LiftState               mliftstate                  = LiftState.LIFT_IDLE; // "m" = class variable that all methods can share. Not a local variable
    private ArmState                marmstate                   = ArmState.ARM_PARKED;
    private DriverCommandState      mdrivercmdstate             = DriverCommandState.UNKNOWN;
    private HomieState              mhomieState                 = HomieState.CENTER;


    // ENUMS
/*
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
*/
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

    private enum HomieState{
        LEFT,
        RIGHT,
        CENTER,
        DRIVER_OPTION
    }
    //DriveSpeedState  currDriveState;
    PatrickState patrickState = PatrickState.OFF;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // initialize the other subsystems
        felipe.init(hardwareMap);
        carousel.init(hardwareMap);
        felipe.armInit();
        felipe.thumbClose();
        felipe.homieCenter();

        double armIPower = 0.2;
        double armDPower = 0.2;


        double juanKfCorr; // corrects feed forward for battery voltage

        int juanPosition = 0; // local variable that only works inside "runOpmode" it is not shared to other methods
        int juanTarget = 0;
        int juanLifError;

        int julioPosition = 0;
        int julioTarget = 0;
        int julioError;

        int changeInArmError        = 0;
        int lastJulioError          = 10;
        int integralError           = 0;


        String lifttimestring ="";
        String armtimestring ="";


        // forces Juan to mechanical low stop and sets encoders to zero
       felipe.juanMechanicalReset();
        // this just changes the state. It does not drive any action

        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );



            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            // Calculations to run once per loop

            juanKfCorr = (felipe.juanKf *felipe.voltSensor.getVoltage()) /12;
            juanPosition = felipe.linearActuator.getCurrentPosition(); // call once before switch to save methods calls.
            juanLifError = juanTarget - juanPosition; // cast as an int because target is a double
            julioPosition = felipe.julioArm.getCurrentPosition(); // call once before switch to save methods calls.
            julioError = (int) (julioTarget - julioPosition); // cast as an int because target is a double


            //////////////////////////////////////////////
            // Gampepad 1 Functions///////////////////////
            /////////////////////////////////////////////

            if (gamepad1.left_bumper && patrickState == PatrickState.OFF) {
                felipe.intakeOn();
                felipe.homieCenter();
                patrickState = PatrickState.COLLECT;
                telemetry.addData("Collector State", patrickState);
                debounce(175); // need to pause for a few ms to let drive release the button

            }
            if (gamepad1.left_bumper && patrickState == PatrickState.COLLECT) {

                felipe.intakeOff();
                patrickState = PatrickState.OFF;
                telemetry.addData("Collector State", patrickState);
                debounce(175);
            }


            if (gamepad1.right_bumper && patrickState == PatrickState.OFF) {

                felipe.intakeEject();
                felipe.homieCenter();
                patrickState = PatrickState.EJECT;

                telemetry.addData("Collector State", patrickState);
                debounce(175);

            }

            if (gamepad1.right_bumper && patrickState == PatrickState.EJECT) {
                felipe.intakeOff();
                patrickState = PatrickState.OFF;

                telemetry.addData("Collector State",patrickState);
                debounce(175);

            }

            /**
             *
             * Gamepad #1  - Buttons       *
             **/




            // Carousel Functions
            if (gamepad1.x) {
                carousel.carouselTurnCCW();
                telemetry.addData("Turning CCW", "Complete ");
            }
            if (gamepad1.b) {
                carousel.carouselTurnCW();
                telemetry.addData("Turning CW", "Complete ");
            }
            if (gamepad1.y) {
                carousel.carouselTurnOff();
                telemetry.addData("Turning Off", "Complete ");
            }

            if (gamepad1.a) {
                felipe.homieCenter();

            }













            /**
             *
             * Gamepad #1 Back Buttons
             *
             **/

            if (gamepad1.back) {
                felipe.julioArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                felipe.julioArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


            //////////////////////////////////////////////
            // Gampepad 2 Functions///////////////////////
            /////////////////////////////////////////////

            /**
             *
             * Gamepad #2  - Buttons       *
             **/



            // Carousel Functions
            if (gamepad2.x) {
                carousel.carouselTurnCCW();
                telemetry.addData("Turning CCW", "Complete ");
            }
            if (gamepad2.b) {
                carousel.carouselTurnCW();
                telemetry.addData("Turning CW", "Complete ");
            }
            if (gamepad2.y) {
                carousel.carouselTurnOff();
                telemetry.addData("Turning Off", "Complete ");
            }

            //double juanpower = -gamepad1.left_stick_y;
            //double juliopower = gamepad1.right_stick_x;


/////////////// Game Pad 1 Assignments /////////////////////////
            // Button names and positions
            //        y
            //      x   b
            //        a


            if (gamepad2.dpad_left) {
                mdrivercmdstate = DriverCommandState.ALLIANCE_HUB_LEFT;
                juanTarget = (int) (felipe.JUANLIFTPARTIAL * felipe.TICKS_PER_LIFT_IN);
                julioTarget = (int) (felipe. JULIOARMLEFT     * felipe.TICKS_PER_DEGREE);
                lifttime.reset();


            }

            if (gamepad2.dpad_right) {
                mdrivercmdstate = DriverCommandState.ALLIANCE_HUB_RIGHT;
                juanTarget = (int) (felipe.JUANLIFTPARTIAL * felipe.TICKS_PER_LIFT_IN);
                julioTarget = (int) (felipe. JULIOARMRIGHT     * felipe.TICKS_PER_DEGREE);
                lifttime.reset();
            }
            if (gamepad2.dpad_down) {
                mdrivercmdstate =  DriverCommandState.RELOAD;
                juanTarget = 0;
                julioTarget = 0;
            }
            // by moving the left joystick in either direction the driver takes manual control.
            if (Math.abs(gamepad2.left_stick_y) > 0.25) {
                mliftstate = LiftState.MANUAL;
            }

            ////////////////////////////  Driver Input Switch Case     ////////////////

            switch (mdrivercmdstate) {

                case ALLIANCE_HUB_LEFT:
                case ALLIANCE_HUB_RIGHT:
                    // This if statement prevents the "Lift Up" being imposed all the time
                    // for example when the lift has transitioned to HOLD, the driver state remains
                    // as ALLIANCE_HUB_LEFT, without this if the state would flip back to "up" and back
                    // to "HOLD" each time through the loop. With the lift, the Driver command has to
                    // be Alliance Hub Left AND the lift has to be at IDLE or Lifting in order to
                    // get that LIF_UP assignment.
                    if (mliftstate == LiftState.LIFT_IDLE | mliftstate == LiftState.LIFT_UP) {
                        mliftstate = LiftState.LIFT_UP;
                    }


                    break;


                case RELOAD:

                    if (marmstate == ArmState.ARM_HOLD | marmstate == ArmState.ARM_CENTER) {
                        marmstate = ArmState.ARM_CENTER;
                    }

                    if (marmstate == ArmState.ARM_PARKED) {
                        mliftstate = LiftState.LIFT_DOWN;
                    }
                    lifttime.reset(); // reset lift time as soon as drive pushes button

                    break;
            } // end of Driver control switch

            ////////////////////////////  Lift State Switch Case     ////////////////

            switch (mliftstate) {

                case LIFT_UP:
                    if (juanPosition < juanTarget){ // this prevents short power up of motors if target
                        // is achieved and the driver accidentally pushes the lift up button again.
                        // without this, the motor will power on for the duration of the loop time; 10-20 ms ot so.
                        //felipe.linearActuator.setPower(juanKfCorr + (juanLifError * felipe.juanKp)); // winch type
                        felipe.linearActuator.setPower(juanKfCorr  +  (juanLifError * felipe.juanKp) ); // screw type
                        lifttimestring = lifttime.toString(); // convert lifttime to a string as long as case is active
                        // the timer never stops. If timer output is converted to text when we wnat, we just read the last value
                        // becase that is all we care about. It tell us how ling we were in this state. Helpful for tuning.
                    }

                    if (juanPosition >= juanTarget) {
                        if (mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_LEFT){


                            armtime.reset();
                            PIDtimer.reset();
                            marmstate = ArmState.ARM_LEFT;
                            felipe.homieLeft();

                        }
                        else if (mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_RIGHT ) {

                            armtime.reset();
                            PIDtimer.reset();
                            marmstate = ArmState.ARM_RIGHT;
                            felipe.homieRight();
                        }
                        mliftstate = LiftState.LIFT_HOLD; // move to the next state
                    }
                    break;

                case LIFT_HOLD: // Power high enough to prevent lift from dropping but not high enough to lift it.
                    felipe.linearActuator.setPower(Math.abs( felipe.JUAN_SPEED_HOLD   )); // requires driver to get out of this state
                    if (mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_LEFT){

                        //mhomieState = HomieState.LEFT;

                    }
                    else if (mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_RIGHT ) {
                        //mhomieState = HomieState.RIGHT;

                    }



                    break;

                case LIFT_DOWN: //
                    if (marmstate != ArmState.ARM_PARKED) {

                        marmstate = ArmState.ARM_CENTER;

                    }
                    else if (marmstate == ArmState.ARM_PARKED){
                        felipe.homieCenter();
                        felipe.linearActuator.setPower(felipe.JUAN_SPEED_DOWN);
                        lifttimestring = lifttime.toString();
                        // The exit condition for LIFT_DOWN is reaching a very low lift position - no operator input needed
                        if (felipe.linearActuator.getCurrentPosition() < 20) {
                            mliftstate = LiftState.LIFT_IDLE;
                        }
                    }

                    break;


                case LIFT_IDLE: //Protects motor by turning it off as the lift is almost at the bottom.

                    felipe.linearActuator.setPower(0);
                    // Exiting this state requires driver button press.
                    break;

                case MANUAL: //Joystick control
                    // Send calculated power to lift
                    //juan.setPower(juanpower);

                    // Exiting this state requires driver button press.
                    break;


            } // end of Lift State switch

            ////////////////////////////  Arm State Switch Case     ////////////////

            switch (marmstate){

                case ARM_LEFT:

                    if (julioPosition > julioTarget){ // only move it below te target
                        felipe.julioArm.setPower(-felipe.JULIO_SPEED_UP);

                    }
                    else{

                        marmstate = ArmState.ARM_HOLD;
                    }

                    break;

                case ARM_RIGHT:
                    if (julioPosition < julioTarget){ // only move it below te target
                        felipe.julioArm.setPower(felipe.JULIO_SPEED_UP);

                    }
                    else{

                        marmstate = ArmState.ARM_HOLD;
                    }
                    break;

                case ARM_HOLD:
                    if(mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_RIGHT) {
                        felipe.julioArm.setPower(felipe.JULIO_SPEED_HOLD + (julioError * felipe.julioKp));
                        //mhomieState = HomieState.DRIVER_OPTION;

                    }
                    if(mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_LEFT) {
                        felipe.julioArm.setPower(-(felipe.JULIO_SPEED_HOLD + (julioError * felipe.julioKp)));
                        //mhomieState = HomieState.DRIVER_OPTION;
                        felipe.hoimeDumpRight();
                    }
                    break;

                case ARM_CENTER: // Center needs to bring to center and hold there. When lift is down, it can go into a park mode.
                    changeInArmError = julioError - lastJulioError; // for the integral term only. Cumulative error tracking
                    integralError = (int)(integralError  + (julioError * PIDtimer.time()));
                    armIPower = felipe.julioKi *  integralError;
                    armDPower = felipe.julioKd * (changeInArmError)/PIDtimer.time();
                    lastJulioError =  integralError;
                    PIDtimer.reset();
                    //juanLifError = juanTarget - juanPosition; // reminder to keep math straight.

                    // Right Case - center with "left/negative" power
                    if ((julioPosition -julioTarget) > 50) {
                        felipe.julioArm.setPower(julioError * felipe.julioKp);//+ armDPower
                    }
                    // Left Case - center with "right/positive" power
                    else if ((julioPosition -julioTarget) < -50){
                        felipe.julioArm.setPower(julioError * felipe.julioKp);
                    }

                    else{
                        marmstate = ArmState.ARM_PARKED;
                    }

                    break;

                case ARM_PARKED:

                    //julio.setPower((julioError * julioKp));
                    felipe.julioArm.setPower((0));

                    break;

            } // end of arm state switch case

            ////////////////////////////  Homie Position Switch Case     ////////////////
            switch (mhomieState){

                case CENTER:
                    felipe.homieCenter();
                    // servos do not respond well here. they are very jerky. Possibly due to repeadly calling
                    // the move servo methods each time through the loop.
                    // call the method above as we exit the lift case so it the servo move is basically called
                    // one time

                    break;

                case LEFT: // the If makes sure the lift is in HOLD before moving the servo.
                    // without the IF the servo has erratic behavior. It seems that the servo methods get called multipe
                    // times during the switch case causing jerky "start-stop" behavior. Possibly due to the
                    // looping through the cases.
                    if (mliftstate == LiftState.LIFT_HOLD){
                        felipe.homieLeft();
                    }

                case RIGHT:
                    if (mliftstate == LiftState.LIFT_HOLD){
                        felipe.homieRight();
                    }

                case DRIVER_OPTION:
                    /**
                     *
                     * Gamepad #1 Triggers - Homie Controls
                     *
                     **/

                    if (gamepad1.right_trigger > 0.25 & felipe.getJulioPosition()<-50) {
                        felipe.homieRight();
                        sleep(500);
                        felipe.homieCenter();
                        sleep(500);
                        //debounce(400);
                        sleep(500);
                        felipe.homieCenter();
                        telemetry.addData("Homie Dump Left", "Complete ");
                    }

                    if (gamepad1.right_trigger > 0.25 & felipe.getJulioPosition()>50) {
                        felipe.homieLeft();
                        sleep(500);
                        felipe.homieCenter();
                        sleep(500);
                        telemetry.addData("Homie Dump Right", "Complete ");


                    }

            }



            // Telemetry to troubleshoot switch cases
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lift Transition Time", lifttimestring);
            telemetry.addData("Arm Transition Time", armtimestring);
            telemetry.addData("Lift State is", mliftstate);
            telemetry.addData("Arm State is", marmstate);
            telemetry.addData("Homie State is", mhomieState);
            telemetry.addData("Driver Cmd State is", mdrivercmdstate);
            telemetry.addData("Lift Encoder Target is", (juanTarget));
            telemetry.addData("Lift Encoder Ticks are", juanPosition);
            telemetry.addData("Arm Encoder Target is", (julioTarget));
            telemetry.addData("Arm Encoder Ticks are", julioPosition);
            telemetry.addData("Motor Power", "Julio (%.2f)", felipe.julioArm.getPower());
            telemetry.addData("Motor Power", "Juan (%.2f)", felipe.linearActuator.getPower());
            telemetry.addData("Battery", "  Voltage (%.2f)", felipe.voltSensor.getVoltage());
            telemetry.update();


        }



        }




    void debounce(long debounceTime){
        try {
            Thread.sleep(debounceTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }






}
