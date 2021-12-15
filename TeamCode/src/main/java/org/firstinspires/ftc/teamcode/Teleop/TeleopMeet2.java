package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.JulioPosition;
import org.firstinspires.ftc.teamcode.Enums.LiftPosition;
import org.firstinspires.ftc.teamcode.Enums.PatrickState;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;

import org.firstinspires.ftc.teamcode.Subsystems.Felipe;
import org.firstinspires.ftc.teamcode.Subsystems.FelipeDeux;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Test")
public class TeleopMeet2 extends LinearOpMode {

    FelipeDeux felipe = new FelipeDeux(this);
    //Felipe felipe = new Felipe(); // instantiate Felipe (the main implement)
    CarouselTurnerThingy carousel = new CarouselTurnerThingy();
    // init and setup
    ElapsedTime runtime = new ElapsedTime();


    // ENUMS
    //DriveSpeedState  currDriveState;
    PatrickState patrickState = PatrickState.OFF;
    LiftPosition liftPosition = LiftPosition.UNKNOWN;

    JulioPosition julioPosition = JulioPosition.CENTER; // states for Julio the Arm

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // initialize the other subsystems
        felipe.init(hardwareMap);
        carousel.init(hardwareMap);
        felipe.armInit();
        felipe.thumbClose();

        double juanErrorMax = 15;
        double juanError    = Math.abs(felipe.getJuanPosition() - felipe.JUANLIFTPARTIAL) ; //current - target

        // forces Juan to mechanical low stop and sets encoders to zero
        felipe.juanMechanicalReset();
        // this just changes the state. It does not drive any action
        liftPosition = LiftPosition.LOAD;
        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        // opmode has to be active to get this method to work. So is won't work if you put it in the init part.
        // This lifts Juan so Hommie is not on the mat.
        felipe.liftLoad();
        // This is the while loop that everything goes inside of.
        // the first section controls the drive train via a roadrunner method
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            //////////////////////////////////////////////
            // Gampepad 1 Functions///////////////////////
            /////////////////////////////////////////////

            /**
             *
             * Gamepad #1 Buttons -
             *
             **/
            if (gamepad1.a) {
                julioPosition = JulioPosition.CENTER;
                telemetry.addData("Reset", "Complete ");
            }

            /**
             *
             * Gamepad #1 Bumpers
             *
             **/

        
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
             * Gamepad #1 DPAD Felipe Controls (setting the States)
             *
             **/
            if (gamepad2.dpad_left) {
                liftPosition = LiftPosition.PARTIAL;
                julioPosition = JulioPosition.LEFT90;
                telemetry.addData("High Goal Left", "Complete");
                telemetry.addData("Lift State",liftPosition);
                telemetry.addData("Arm State", julioPosition);
            }
            if (gamepad2.dpad_right) {
                liftPosition = LiftPosition.PARTIAL;
                julioPosition = JulioPosition.RIGHT90;
                telemetry.addData("High Goal Right","Complete");
            }
            if (gamepad2.dpad_down) {//this one works
                julioPosition = JulioPosition.CENTER;
                liftPosition = LiftPosition.LOAD;
            }

            if (gamepad2.dpad_up) {//this one works
                liftPosition = LiftPosition.UP;
            }




            /**
             *
             * Gamepad #1 Triggers - Homie Controls
             *
             **/

            if (gamepad1.right_trigger > 0.25) {
                felipe.homieLeft();
                sleep(500);
                felipe.homieCenter();
                sleep(500);
                //debounce(400);
                sleep(500);
                felipe.homieCenter();
                telemetry.addData("Homie Left", "Complete ");

                //debounce(400);
            }
            if (gamepad1.left_trigger > 0.25) {
                felipe.homieRight();
                sleep(500);
                felipe.homieCenter();
                sleep(500);

                telemetry.addData("Homie Right", "Complete ");
            }
            /**
             *
             * Gamepad #1 Back Buttons
             *
             **/

            if (gamepad1.back) {
                felipe.julioArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  }


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

            if (gamepad2.back) {
                felipe.julioArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  }

            /**
             *
             * Gamepad #2  - DPAD       *
             **/

/*
            if (gamepad2.dpad_left) {
                liftPosition = LiftPosition.PARTIAL;
                julioPosition = JulioPosition.LEFT45;
                telemetry.addData("High Goal Left", "Complete");
                telemetry.addData("Lift State",liftPosition);
                telemetry.addData("Arm State", julioPosition);
            }
            if (gamepad2.dpad_right) {
                liftPosition = LiftPosition.PARTIAL;
                julioPosition = JulioPosition.RIGHT45;
                telemetry.addData("High Goal Right","Complete");
            }
            if (gamepad2.dpad_down) {//this one works
                julioPosition = JulioPosition.CENTER;
                liftPosition = LiftPosition.LOAD;
            }

            if (gamepad2.dpad_up) {//this one works
                liftPosition = LiftPosition.UP;
            }
*/
            /////////////////////////////////////////////////////////////////////////
            //States and Trasnitions//////////////////////////////////////////////
            if  (liftPosition == LiftPosition.PARTIAL && julioPosition == JulioPosition.LEFT90) {// where we need to go
                telemetry.addData("Going to Lift PARTIAL and Left 90 Degrees", "Done");

                //this sets the target when the button is pressed as the new STATE is set



                if( felipe.getJuanPosition() < felipe.JUANLIFTPARTIAL ){
                    felipe.setJuanToPartial();
                    felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.linearActuator.setPower(Math.abs(felipe.JUANLIFTSPEED ));
                }
                //only after linear actuator reaches target height does the julio start moving
                if( felipe.getJuanPosition() >= felipe.JUANLIFTPARTIAL){
                    felipe.setJulioTo90Left();
                    felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.julioArm.setPower(Math.abs(felipe.JULIOTURNSPEED));
                }

                telemetry.addData("Lift State",  liftPosition);
                telemetry.addData("Lift Position (inches)", felipe.getJuanPosition());
                telemetry.update();
            }




            //if the dpad right is pressed, the code below gets executed
            if  (liftPosition == LiftPosition.PARTIAL && julioPosition == JulioPosition.RIGHT90) {// where we need to go
                telemetry.addData("Going to Lift PARTIAL and Right 90 Degrees", "Done");

                //this sets the target when the button is pressed as the new STATE is set


                if( felipe.getJuanPosition() < felipe.JUANLIFTPARTIAL  ){
                    felipe.setJuanToPartial();
                    felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.linearActuator.setPower(Math.abs(felipe.JULIOTURNSPEED));
                }
                //only after linear actuator reaches target height does the julio start moving
                if( felipe.getJuanPosition() >= felipe.JUANLIFTPARTIAL  ){
                    felipe.setJulioTo90Right();
                    felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.julioArm.setPower(Math.abs(felipe.JULIOTURNSPEED));
                }

                telemetry.addData("Lift State",  liftPosition);
                telemetry.addData("Lift Position (inches)", felipe.getJuanPosition());
                telemetry.update();
            }


            //if the dpad down is pressed, the code below gets executed
            if  (liftPosition == LiftPosition.LOAD && julioPosition == JulioPosition.CENTER) {// where we need to go
                telemetry.addData("Going to Lift PARTIAL and Right 90 Degrees", "Done");

                //this sets the target when the button is pressed as the new STATE is set





                if((juanError < juanErrorMax) && Math.abs(felipe.getJulioPosition()) >= 15) {

                    felipe.rotateToTargetAngle(0,1);
                    //felipe.setJulioToZero();
                    //felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //felipe.julioArm.setPower(Math.abs(felipe.JULIOTURNSPEED));

                    telemetry.addData("Lift State", liftPosition);
                    telemetry.addData("Arm Only Moving . Cunnet degrees", felipe.getJuanPosition());

                }


                if( (juanError < juanErrorMax) && Math.abs(felipe.getJulioPosition()) < 15) {

                    felipe.setJuanToLoad();
                    felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);



                    telemetry.addData("Lift State", liftPosition);
                    telemetry.addData("Arm Only Moving . Cunnet degrees", felipe.getJuanPosition());

                }




            }




            /////////////////////////////////////////////////////////////////////////
            //States and Trasnitions//////////////////////////////////////////////
            if  (liftPosition == LiftPosition.PARTIAL && julioPosition == JulioPosition.LEFT45) {// where we need to go
                telemetry.addData("Going to Lift PARTIAL and Left 45 Degrees", "Done");

                //this sets the target when the button is pressed as the new STATE is set



                if( felipe.getJuanPosition() < felipe.JUANLIFTPARTIAL ){
                    felipe.setJuanToPartial();
                    felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.linearActuator.setPower(Math.abs(felipe.JUANLIFTSPEED ));
                }
                //only after linear actuator reaches target height does the julio start moving
                if( felipe.getJuanPosition() >= felipe.JUANLIFTPARTIAL){
                    felipe.setJulioTo45Left();
                    felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.julioArm.setPower(Math.abs(felipe.JULIOTURNSPEED));
                }

                telemetry.addData("Lift State",  liftPosition);
                telemetry.addData("Lift Position (inches)", felipe.getJuanPosition());
                telemetry.update();
            }




            //if the dpad right is pressed, the code below gets executed
            if  (liftPosition == LiftPosition.PARTIAL && julioPosition == JulioPosition.RIGHT45) {// where we need to go
                telemetry.addData("Going to Lift PARTIAL and Right 90 Degrees", "Done");

                //this sets the target when the button is pressed as the new STATE is set


                if( felipe.getJuanPosition() < felipe.JUANLIFTPARTIAL  ){
                    felipe.setJuanToPartial();
                    felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.linearActuator.setPower(Math.abs(felipe.JULIOTURNSPEED));
                }
                //only after linear actuator reaches target height does the julio start moving
                if( felipe.getJuanPosition() >= felipe.JUANLIFTPARTIAL  ){
                    felipe.setJulioTo45Right();
                    felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.julioArm.setPower(Math.abs(felipe.JULIOTURNSPEED));
                }

                telemetry.addData("Lift State",  liftPosition);
                telemetry.addData("Lift Position (inches)", felipe.getJuanPosition());
                telemetry.update();
            }


            //if the dpad down is pressed, the code below gets executed
            if  (liftPosition == LiftPosition.LOAD && julioPosition == JulioPosition.CENTER) {// where we need to go
                telemetry.addData("Going to Lift PARTIAL and Right 90 Degrees", "Done");

                //this sets the target when the button is pressed as the new STATE is set





                if((juanError < juanErrorMax) && Math.abs(felipe.getJulioPosition()) >= 15) {

                    felipe.rotateToTargetAngle(0,1);
                    //felipe.setJulioToZero();
                    //felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //felipe.julioArm.setPower(Math.abs(felipe.JULIOTURNSPEED));

                    telemetry.addData("Lift State", liftPosition);
                    telemetry.addData("Arm Only Moving . Cunnet degrees", felipe.getJuanPosition());

                }


                if( (juanError < juanErrorMax) && Math.abs(felipe.getJulioPosition()) < 15) {

                    felipe.setJuanToLoad();
                    felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);



                    telemetry.addData("Lift State", liftPosition);
                    telemetry.addData("Arm Only Moving . Cunnet degrees", felipe.getJuanPosition());

                }




            }

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
