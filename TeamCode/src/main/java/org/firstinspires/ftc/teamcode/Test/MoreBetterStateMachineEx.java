 package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 @TeleOp(name="Meet 3 Teleop Modified FSM",group = "Test")
 public class MoreBetterStateMachineEx extends LinearOpMode {

     private Felipe5                felipe                      = new Felipe5(this);
     private CarouselTurnerThingy    carousel                   = new CarouselTurnerThingy();
     private ElapsedTime             runtime                    = new ElapsedTime();
     private ElapsedTime             lifttime                   = new ElapsedTime(); // used to see how long it takes to lift
     private ElapsedTime             armtime                    = new ElapsedTime(); // used to see how long it takes the arm to pivot
     private ElapsedTime             PIDtimer                   = new ElapsedTime(); // used to see how long it takes the arm to pivot
     private ElapsedTime             teleopTimer                = new ElapsedTime();

     private LiftState               mliftstate                 = LiftState.LIFT_IDLE; // "m" = class variable that all methods can share. Not a local variable
     private ArmState                marmstate                  = ArmState.ARM_PARKED;
     private DriverCommandState      mdrivercmdstate            = DriverCommandState.UNKNOWN;
     private HomieState              mhomieState                = HomieState.NONE;

     private int                TELEOP_TIME_OUT                 = 125;

     // ENUMS

     private enum LiftState {
         LIFT_DOWN,

         LIFT_UP_TARGET_SET,
         LIFT_MOVING_UP,
         LIFT_HOLD_UP,
         LIFT_DOWN_TARGET_SET,
         LIFT_MOVING_DOWN,

         LIFT_IDLE,
         MANUAL,
         LIFT_MECH_RESET,
         UNKNOWN
     }

     private enum ArmState {
         ARM_CENTER,
         ARM_FINAL_CENTER,
         ARM_LEFT_TARGET_SET,
         ARM_RIGHT_TARGET_SET,
         ARM_LEFT_ROTATE,
         ARM_RIGHT,
         ARM_HOLD,
         ARM_PARKED,
         MANUAL,
         ARM_IDLE,
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
         DRIVER_OPTION,
         NONE
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
         double julioParkPowerCOrr;
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
        telemetry.addData("Lift Resetting", "Complete");
        telemetry.update();
         // this just changes the state. It does not drive any action

         ////////////////////////////////////////////////////////////////////////////////////////////
         // WAIT FOR MATCH TO START
         ///////////////////////////////////////////////////////////////////////////////////////////
         waitForStart();
         felipe.liftLoad();

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
                     // This if statement prevents repeated setting the target
                     if (mliftstate != LiftState.LIFT_UP_TARGET_SET) {
                         mliftstate = LiftState.LIFT_UP_TARGET_SET;
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

                 case LIFT_UP_TARGET_SET:
                     felipe.setJuanToPartial();
                     felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     felipe.linearActuator.setPower(felipe.JUANLIFTSPEED);
                     mliftstate =  LiftState.LIFT_MOVING_UP;
                     break;

                 case LIFT_MOVING_UP: // Power high enough to prevent lift from dropping but not high enough to lift it.

                     if (felipe.linearActuator.isBusy()){
                        // Do nothing....just let the lift finish lifting
                         // lift should stay up here as well.

                     }
                     else  {
                         if(mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_RIGHT & marmstate != ArmState.ARM_RIGHT_TARGET_SET) {
                            marmstate = ArmState.ARM_RIGHT_TARGET_SET;

                         }
                         if(mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_LEFT & marmstate != ArmState.ARM_LEFT_TARGET_SET) {
                             marmstate = ArmState.ARM_LEFT_TARGET_SET;

                         }// the only way out of this state is a drive command
                         mliftstate =  LiftState.LIFT_HOLD_UP;

                      }
                      break;

                 case LIFT_HOLD_UP:
                     felipe.linearActuator.setPower(felipe.JUAN_SPEED_HOLD);// do we need this?
                     break;



                 case LIFT_DOWN_TARGET_SET:
                     felipe.setJuanToLoad();
                     felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     felipe.linearActuator.setPower(felipe.JUANLIFTSPEED);
                     mliftstate =  LiftState.LIFT_MOVING_DOWN;


                     break;


                 case LIFT_MOVING_DOWN: //Protects motor by turning it off as the lift is almost at the bottom.
                     if (felipe.linearActuator.isBusy()){

                     }
                     else  {
                         felipe.linearActuator.setPower(0);
                     }

                     break;

                 case MANUAL: //Joystick control
                     // Send calculated power to lift
                     //juan.setPower(juanpower);

                     // Exiting this state requires driver button press.
                     break;


             } // end of Lift State switch

             ////////////////////////////  Arm State Switch Case     ////////////////

             switch (marmstate){

                 case ARM_LEFT_TARGET_SET:
                     felipe.setJulioTo90Left();
                     felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     felipe.julioArm.setPower(felipe.JULIO_SPEED_UP);
                     marmstate =  ArmState.ARM_LEFT_ROTATE;
                     break;

                 case ARM_LEFT_ROTATE:

                     if (felipe.julioArm.isBusy()){
                        // do nothing while arm is moving. Leave it out there when target is reached
                     }


                     break;

                 case ARM_HOLD:
                     if(mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_RIGHT) {

                     }
                     if(mdrivercmdstate == DriverCommandState.ALLIANCE_HUB_LEFT) {


                     }
                     break;

                 case ARM_CENTER: // Center needs to bring to center and hold there. When lift is down, it can go into a park mode.


                     break;
                 

             } // end of arm state switch case

             ////////////////////////////  Homie Position Switch Case     ////////////////
             switch (mhomieState){

                 case CENTER:
                     //felipe.homieCenter();
                     // servos do not respond well here. they are very jerky. Possibly due to repeadly calling
                     // the move servo methods each time through the loop.
                     // call the method above as we exit the lift case so it the servo move is basically called
                     // one time

                     break;

                 case LEFT: // the If makes sure the lift is in HOLD before moving the servo.
                     // without the IF the servo has erratic behavior. It seems that the servo methods get called multipe
                     // times during the switch case causing jerky "start-stop" behavior. Possibly due to the
                     // looping through the cases.
                     //felipe.homieLeft();


                 case RIGHT:
                     //if (mliftstate == LiftState.LIFT_HOLD){
                       //  felipe.homieRight();
                     //}

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
             //telemetry.addData("Lift Transition Time", lifttimestring);
             //telemetry.addData("Arm Transition Time", armtimestring);
             telemetry.addData("Lift State is", mliftstate);
             telemetry.addData("Arm State is", marmstate);
             //telemetry.addData("Homie State is", mhomieState);
             telemetry.addData("Driver Cmd State is", mdrivercmdstate);
             telemetry.addData("Lift Encoder Target is", (juanTarget));
             telemetry.addData("Lift Encoder Ticks are", juanPosition);
             telemetry.addData("Arm Encoder Target is", (julioTarget));
             telemetry.addData("Arm Encoder Ticks are", julioPosition);

             telemetry.addData("Arm Change in Error",  changeInArmError);
             telemetry.addData("Arm Cumulative Error",  integralError);
             telemetry.addData("PID Time Duration",  PIDtimer.time()); // about 0.007 to 0.008 seconds
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
