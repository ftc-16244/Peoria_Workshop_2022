 package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.JulioPosition;
import org.firstinspires.ftc.teamcode.Enums.LiftState;
import org.firstinspires.ftc.teamcode.Enums.PatrickState;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.Felipe4;
import org.firstinspires.ftc.teamcode.Subsystems.Felipe5;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


 /**
  * This is a simple teleop routine for testing localization. Drive the robot around like a normal
  * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
  * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
  * exercise is to ascertain whether the localizer has been configured properly (note: the pure
  * encoder localizer heading may be significantly off if the track width has not been tuned).
  */
 @TeleOp(name="More Better State Machine Test",group = "Test")
 public class MoreBetterStateMachineEx extends LinearOpMode {
     private final FtcDashboard dashboard = FtcDashboard.getInstance();
     private Felipe5                 felipe                      = new Felipe5(this);
     private CarouselTurnerThingy    carousel                   = new CarouselTurnerThingy();
     private ElapsedTime             runtime                    = new ElapsedTime();
     private ElapsedTime             lifttime                   = new ElapsedTime(); // used to see how long it takes to lift
     private ElapsedTime             armtime                    = new ElapsedTime(); // used to see how long it takes the arm to pivot
     private ElapsedTime             PIDtimer                   = new ElapsedTime(); // used to see how long it takes the arm to pivot
     private ElapsedTime             teleopTimer                = new ElapsedTime();

     private LiftState               mliftstate                 = LiftState.LIFT_IDLE; // "m" = class variable that all methods can share. Not a local variable
     private ArmState                marmstate                  = ArmState.UNKNOWN;
     private DriverCommandState      mdrivercmdstate            = DriverCommandState.UNKNOWN;


     private int                TELEOP_TIME_OUT                 = 125;

     public static final double NEW_P = 7;//10 orig
     public static final double NEW_I = 1.0;//0.5
     public static final double NEW_D = 0.0;//0.5
     public static final double NEW_F = 0;//10
     public int tol;
     public int newtolerance = 3;





     // ENUMS

     private enum LiftState {
         LIFT_DOWN,
         LIFT_UP_AND_HOLD,
         LIFT_IDLE,
         MANUAL,
         LIFT_SHARED_HUB_DROP,
         LIFT_MECH_RESET,
         UNKNOWN
     }

     private enum ArmState {
         ARM_CENTER_ROTATE,
         ARM_LEFT_ALLIANCE,
         ARM_RIGHT_ALLIANCE,
         ARM_LEFT_SHARRED,
         ARM_RIGHT_SHARRED,
         ARM_IDLE,
         UNKNOWN

     }

     private enum DriverCommandState {
         RELOAD,
         ALLIANCE_HUB_LEFT,
         ALLIANCE_HUB_RIGHT,
         SHARED_HUB_LEFT,
         SHARED_HUB_RIGHT,
         UNKNOWN
     }



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

         // local variable that only works inside "runOpmode" it is not shared outside

         int juanPosition = 0;
         int juanTarget = 0;
         int juanLifError;

         int julioPosition = 0;
         int julioTarget = 0;
         int julioError;


         String lifttimestring ="";
         String armtimestring ="";

        telemetry.addData("Lift Resetting", "Complete");
        telemetry.update();
         // this just changes the state. It does not drive any action

         // get the PID coefficients for the RUN_USING_ENCODER  modes.
         PIDFCoefficients pidfOrig = felipe.julioArm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

         // change coefficients using methods included with DcMotorEx class.
         PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
         felipe.julioArm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

         // re-read coefficients and verify change.
         PIDFCoefficients pidModified = felipe.julioArm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

         tol = felipe.julioArm.getTargetPositionTolerance();
         felipe.julioArm.setTargetPositionTolerance(newtolerance);

         telemetry.addData("Runtime", "%.03f", getRuntime());
         telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.04f, %.04f",
                 pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
         telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                 pidModified.p, pidModified.i, pidModified.d, pidModified.f);
         telemetry.addData("Encoder Tolerance",  tol);
         telemetry.update();

         telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());


         // forces Juan to mechanical low stop and sets encoders to zero
         felipe.juanMechanicalReset();// commented out for testing. Remove comment for competiton
         ////////////////////////////////////////////////////////////////////////////////////////////
         // WAIT FOR MATCH TO START
         ///////////////////////////////////////////////////////////////////////////////////////////
         waitForStart();
         felipe.liftLoad(); // commented out for testing. Remove comment for competiton

         telemetry.addData("Encoder Reset to ",felipe.linearActuator.getCurrentPosition());

         while (!isStopRequested()  &&  teleopTimer.seconds() < TELEOP_TIME_OUT ) {
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

             // Helper variables computed each loop

             juanPosition       = felipe.linearActuator.getCurrentPosition(); // call once before switch to save methods calls.
             juanLifError       = juanTarget - juanPosition; // target is a double, position in an int
             julioPosition      = felipe.julioArm.getCurrentPosition(); // call once before switch to save methods calls.
             julioError         = julioTarget - julioPosition;

             //////////////////////////////////////////////
             // Gampepad 1 Functions///////////////////////
             /////////////////////////////////////////////

             /**
              *
              * Gamepad #1  - Bumpers - Intake
              **/
             if (gamepad1.left_bumper && patrickState == PatrickState.OFF) {
                 felipe.intakeOn();
                 felipe.homieCenter();
                 patrickState = PatrickState.COLLECT;
                 telemetry.addData("Collector State", patrickState);
                 debounce(175); // need to pause for a few ms to let driver release the button

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
              * Gamepad #1  - Buttons Shared Hub controls       *
              **/

             if (gamepad1.x && mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_LEFT) {
                 mdrivercmdstate = DriverCommandState. SHARED_HUB_LEFT;
             }
             if (gamepad1.b && mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_RIGHT) {
                 mdrivercmdstate = DriverCommandState. SHARED_HUB_RIGHT;
             }
             if (gamepad1.y) {

             }

             if (gamepad1.a) {


             }
             /**
              *
              * Gamepad #1 Back Button
              *
              **/

             if (gamepad1.back) {
                 felipe.julioArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                 felipe.julioArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             }
            /**
             *
             **/

             if (gamepad1.right_trigger > 0.25 && (mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_LEFT | mdrivercmdstate == DriverCommandState.SHARED_HUB_LEFT)) {
                 felipe.homieRight();
                 sleep(500);
                 felipe.homieCenter();
                 sleep(500);
                 //debounce(400);
                 sleep(500);
                 felipe.homieCenter();
                 telemetry.addData("Homie Dump Left", "Complete ");
             }

             if (gamepad1.right_trigger > 0.25 && (mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_RIGHT | mdrivercmdstate == DriverCommandState.SHARED_HUB_RIGHT)) {
                 felipe.homieLeft();
                 sleep(500);
                 felipe.homieCenter();
                 sleep(500);
                 telemetry.addData("Homie Dump Right", "Complete ");


             }


             //////////////////////////////////////////////
             // Gampepad 2 Functions///////////////////////
             /////////////////////////////////////////////

             /**
              *
              * Gamepad #2  - Buttons       *
              **/

             // Button names and positions
             //        y
             //      x   b
             //        a

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


             /**
              *
              * Gamepad #2  - Dpad - Lift and Arm Control      *
              **/

             if (gamepad2.dpad_left) {
                 mdrivercmdstate = DriverCommandState.ALLIANCE_HUB_LEFT;
             }

             if (gamepad2.dpad_right) {
                 mdrivercmdstate = DriverCommandState.ALLIANCE_HUB_RIGHT;
             }
             if (gamepad2.dpad_down) {
                 mdrivercmdstate =  DriverCommandState.RELOAD;
             }

             /**
              *
              * Gamepad #2  - Dpad - Homie Rotate Control      *
              **/
             if (gamepad2.left_trigger > 0.25 && felipe.linearActuator.getCurrentPosition() > 600 && mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_LEFT) {
                 felipe.homieRight();
             }

             if (gamepad2.left_trigger > 0.25 &&  felipe.linearActuator.getCurrentPosition() > 600 && mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_RIGHT) {
                 felipe.homieLeft();
             }

             ////////////////////////////  Driver Input Switch Case     ////////////////

             switch (mdrivercmdstate) {

                 case ALLIANCE_HUB_LEFT:
                 case ALLIANCE_HUB_RIGHT:
                     // This if statement prevents repeatedly setting the target and run to positions
                     if (mliftstate != LiftState.LIFT_UP_AND_HOLD) {
                         mliftstate = LiftState.LIFT_UP_AND_HOLD;
                     }
                     break;

                 case RELOAD:
                     if (mliftstate != LiftState.LIFT_DOWN) {
                         marmstate = ArmState.ARM_CENTER_ROTATE; // this has to go first
                     }
                     break;
                 case SHARED_HUB_LEFT:
                     // This if statement makes sure the arm is on the left before lowering. It prevents crashes
                     if (marmstate == ArmState.ARM_LEFT_ALLIANCE) {
                         marmstate = ArmState.ARM_LEFT_SHARRED;
                     }
                     break;

                 case SHARED_HUB_RIGHT:
                     // This if statement makes sure the arm is on the right before lowering. It prevents crashes
                     if (marmstate == ArmState.ARM_RIGHT_ALLIANCE) {
                         marmstate = ArmState.ARM_RIGHT_SHARRED;
                     }
                     break;


             } // end of Driver control switch

             ////////////////////////////  Lift State Switch Case     ////////////////

             switch (mliftstate) {
                 // Lift an hold are combined because RUN TO POSITION does just that.
                 // This case is only accessed the first loop after the driver selects "left alliance"
                 // because don't want to keep repeating the target set and run to position each time through the while loop.

                 case LIFT_UP_AND_HOLD:
                     felipe.setJuanToPartial(); // method that sets target position
                     juanTarget = (int) (felipe.JUANLIFTPARTIAL * felipe.TICKS_PER_LIFT_IN); // for telemetry
                     felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     felipe.linearActuator.setPower(felipe.JUANLIFTSPEED);

                     break;

                 case LIFT_DOWN:
                     felipe.setJuanToLoad();
                     juanTarget = (int) (felipe.JUANLIFTLOAD * felipe.TICKS_PER_LIFT_IN); // for telemetry
                     felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     felipe.linearActuator.setPower(felipe.JUANLIFTSPEED);



                     break;

                 case LIFT_IDLE:
                     // Not used yet

                     break;

             } // end of Lift State switch

             ////////////////////////////  Arm State Switch Case     ////////////////

             switch (marmstate){

                 case  ARM_LEFT_ALLIANCE:
                     felipe.julioArm.setTargetPosition((int) (felipe.JULIOARMLEFT * felipe.TICKS_PER_DEGREE));
                     julioTarget = (int) (felipe.JULIOARMLEFT * felipe.TICKS_PER_DEGREE); // for telemetry
                     felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     felipe.julioArm.setPower(felipe.JULIO_SPEED_UP);

                     break;
                 case  ARM_RIGHT_ALLIANCE:

                     felipe.julioArm.setTargetPosition((int)(felipe.JULIOARMRIGHT * felipe.TICKS_PER_DEGREE));
                     julioTarget = (int) (felipe.JULIOARMRIGHT * felipe.TICKS_PER_DEGREE); // for telemetry
                     felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     felipe.julioArm.setPower(felipe.JULIO_SPEED_UP);

                     break;

                 case  ARM_CENTER_ROTATE:

                     felipe.julioArm.setTargetPosition((int)(felipe.JULIOARMCENTER * felipe.TICKS_PER_DEGREE));
                     julioTarget = (int) (felipe. JULIOARMCENTER  * felipe.TICKS_PER_DEGREE); // for telemetry it is just zero
                     felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     felipe.julioArm.setPower(felipe.JULIO_SPEED_DOWN);
                     felipe.intakeOff();
                     patrickState = PatrickState.OFF;


                     break;

                 case  ARM_IDLE:

                     felipe.julioArm.setPower(0);

                     //cant put intake off here becase it will not turn on with driver control

                     break;

                 case  ARM_LEFT_SHARRED:

                     felipe.julioArm.setTargetPosition((int)(felipe.JULIOARMLEFT45 * felipe.TICKS_PER_DEGREE));
                     julioTarget = (int) (felipe.JULIOARMLEFT45 * felipe.TICKS_PER_DEGREE); // for telemetry
                     felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     felipe.julioArm.setPower(felipe.JULIO_SPEED_UP);


                     break;
                 case  ARM_RIGHT_SHARRED:

                     felipe.julioArm.setTargetPosition((int)(felipe.JULIOARMRIGHT45 * felipe.TICKS_PER_DEGREE));
                     julioTarget = (int) (felipe.JULIOARMRIGHT45 * felipe.TICKS_PER_DEGREE); // for telemetry
                     felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     felipe.julioArm.setPower(felipe.JULIO_SPEED_UP);


                     break;





             } // end of arm state switch case


             ////////////////////////////  Lift and Arm position and state checks to sequence the movements correctly and avoid crashes.     ////////////////

             // when lift is no longer busy (! symbol), go ahead and advance the arm to the rotate state.
             // The arm lowers the freight box "Homie" in between the frame rails. The arm will hit the frame
             // unless it is lifted up about 6 inches first. THus a lot of sequencing is required.
             if(felipe.linearActuator.getCurrentPosition() > 2000 && mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_LEFT) {
                 marmstate = ArmState.ARM_LEFT_ALLIANCE;
             }

             if(!felipe.linearActuator.isBusy() && mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_RIGHT) {
                 marmstate = ArmState.ARM_RIGHT_ALLIANCE; // when lift is no longer busy (! symbol), go ahead and advance the arm to the rotate state.
             }

             // when RELOAD is called for, don't lower the lift until arm is centered.
             if(Math.abs(felipe.julioArm.getCurrentPosition()) < 10 && mdrivercmdstate == DriverCommandState.RELOAD) {
                 mliftstate = LiftState.LIFT_DOWN;
             }
            // When arm is in the center, the driver is in reload, and the actuator is near the bottom. Kill power to arm to protect it.
             if(Math.abs(felipe.julioArm.getCurrentPosition()) <= 6 && felipe.linearActuator.getCurrentPosition() <250 && mdrivercmdstate == DriverCommandState.RELOAD) {
                 marmstate = ArmState.ARM_IDLE;
             }


             //if(felipe.linearActuator.getCurrentPosition() > 375 && felipe.linearActuator.getCurrentPosition() < 800 && mdrivercmdstate == DriverCommandState.RELOAD) {
               //  felipe.intakeHelpHomie();
             //}

             //if(felipe.linearActuator.getCurrentPosition() > 300 && felipe.linearActuator.getCurrentPosition() < 350 && mdrivercmdstate == DriverCommandState.RELOAD) {
               //  felipe.intakeOff();
             //}

             if(Math.abs(julioError) <= 20 && mdrivercmdstate == DriverCommandState.SHARED_HUB_LEFT | mdrivercmdstate == DriverCommandState.SHARED_HUB_RIGHT){
                mliftstate = LiftState.LIFT_DOWN;
             }


             // For a cascade lift, add a kill power also. We dont need that command with the linear actuator.

             ////////////////////////////  Homie Position Switch Case     ////////////////

             // Telemetry to troubleshoot switch cases
             //telemetry.addData("Status", "Run Time: " + runtime.toString());
             //telemetry.addData("Lift Transition Time", lifttimestring);
             //telemetry.addData("Arm Transition Time", armtimestring);
             telemetry.addData("Lift State is", mliftstate);
             telemetry.addData("Arm State is", marmstate);
             telemetry.addData("Lift Error in Ticks", (juanTarget - felipe.linearActuator.getCurrentPosition()));
             telemetry.addData("Lift Error in Ticks-loop", (juanLifError));
             telemetry.addData("Driver Cmd State is", mdrivercmdstate);
             telemetry.addData("Lift Encoder Target is", (juanTarget));
             telemetry.addData("Lift Encoder Ticks are", juanPosition);
             telemetry.addData("Arm Encoder Target is", (julioTarget));
             telemetry.addData("Arm Encoder Ticks are", julioPosition);
             telemetry.addData("Arm Error in Ticks are", julioError);
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
