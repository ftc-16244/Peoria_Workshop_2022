package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Enums.SlideTrainerState;
import org.firstinspires.ftc.teamcode.Subsystems.Slide_Trainer;
import org.firstinspires.ftc.teamcode.Subsystems.Slide_Trainer_Seq;

/**
 * This is the next improvment on the Viper Slide Trainer. Here we coordiante the lift and servo
 * commands all onto the buttone. Each button moves the slide to a level and then moved the servo
 * to one of 4 positions. This is pretty good and in many cases good enough. This opmode has a pitfall
 * that can be frustration. The slide sits in a while loop until it reaches its target. Once the target is
 * reached, the servo moves. This is good for lift an servo cordination. But, this while loop cases the
 * other gamepad functions to be "suspended" until the loop finishes. This is terrible for coordinating
 * the drivetrain with an implement. The drivetrain power will stay constant while the slide is in its loop
 * this means inability to start or stop at times. Use with caution. Or go to the state machine version.
 */

@Config
@TeleOp(group = "Viper Slide w/ Sequencing")
//@Disabled
public class C_SlideTrainerMoreAuto<slideTrainerState> extends LinearOpMode {

    FtcDashboard dashboard;

    Slide_Trainer_Seq slideTrainer = new Slide_Trainer_Seq(this);
    SlideTrainerState slideTrainerState = SlideTrainerState.UNKNOWN;

    double pos;
    //public static double SLIDE_NEW_P = 2.5; // 2.5
    //public static double SLIDE_NEW_I = 0.1;// 0.1
    //public static double SLIDE_NEW_D = 0.2; // 0.2
    //public static double SLIDE_NEW_F = 10; // 10

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize HW by calling the init method stored in the subsystem
        slideTrainer.init(hardwareMap);

        slideTrainer.slideMechanicalReset(); // run reset on init to make sure slide is retracted all the way\
        slideTrainerState = SlideTrainerState.MECH_RESET; // init puts us in this state, the timew and limit swtch tell when we coe out of it.
        telemetry.addData("Lift State", slideTrainerState);
        dashboard = FtcDashboard.getInstance();

        float random_power =0;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////



        waitForStart();

        while (!isStopRequested()) {


            // Gampepad 1 Functions
            random_power = -gamepad1.left_stick_y;
            slideTrainer.randomMotor.setPower(random_power);

            /**
             *
             * Gamepad #1 Buttons -
             *  // Button names and positions
             *                      y
             *                    x   b
             *                      a
             *
             **/
            if (gamepad1.a) {
                slideTrainer.setSlideLevel4();
                slideTrainer.servo.setPosition(.25);
                slideTrainerState = SlideTrainerState.EXTRA_HIGH;
            }

            if (gamepad1.b) {

                slideTrainer.setSlideLevel3();
                slideTrainer.servo.setPosition(.5);
                slideTrainerState = SlideTrainerState.HIGH;

            }

            if (gamepad1.x) {

                slideTrainer.setSlideLevel1();
                slideTrainer.servo.setPosition(.75);
                slideTrainerState = SlideTrainerState.LOW;

            }

            if (gamepad1.y) {

                slideTrainer.setSlideLevel2();
                slideTrainer.servo.setPosition(1.0);
                slideTrainerState = SlideTrainerState.MID;

            }



            /**
             *
             * Gamepad #1 Bumpers
             *
             **/

        
            if (gamepad1.left_bumper) {



            }
            if (gamepad1.left_bumper) {



                debounce(175);
            }


            if (gamepad1.right_bumper) {





            }

            if (gamepad1.right_bumper) {

                debounce(175);

            }

            /**
             *
             * Gamepad #1 Back Button
             *
             **/
            if (gamepad1.back) {

               slideTrainer.slideMechanicalReset();
               slideTrainer.targetHeight = 0;
               slideTrainerState =  SlideTrainerState.MECH_RESET;
            }

            /**
             *
             * Gamepad #1 DPAD
             *
             **/
            if (gamepad1.dpad_left) {

                telemetry.addData("High Goal", "Complete ");
            }

            if (gamepad1.dpad_right) {

                telemetry.addData("High Goal", "Complete ");
            }

            if (gamepad1.dpad_down) {

                telemetry.addData("lower elevator", "Complete ");
            }

            /**
             *
             * Gamepad #1 Triggers
             *
             **/

            if (gamepad1.left_trigger > 0.25) {

                //debounce(400);
                telemetry.addData("Homie Left", "Complete ");

                //debounce(400);
            }
            if (gamepad1.right_trigger > 0.25) {

                telemetry.addData("Homie Right", "Complete ");
            }

            switch (slideTrainerState){
                case UNKNOWN:
                    break;
                case IDLE:
                    break;

                case LOW:
                    telemetry.addData("Low Slide Position", slideTrainerState);
                    slideTrainer.greenLED.setState(false);
                    slideTrainer.redLED.setState(true);

                    break;

                case MID:
                    telemetry.addData("Mid Slide Position", slideTrainerState);
                    slideTrainer.greenLED.setState(false);
                    slideTrainer.redLED.setState(false);

                    break;
                case HIGH:
                    telemetry.addData("High Slide Position", slideTrainerState);
                    slideTrainer.greenLED.setState(true);
                    slideTrainer.redLED.setState(true);
                    break;
                case EXTRA_HIGH:
                    telemetry.addData("Extra high Position", slideTrainerState);
                    slideTrainer.greenLED.setState(true);
                    slideTrainer.redLED.setState(false);
                    break;
                case MECH_RESET:
                    telemetry.addData("Hard Reset", slideTrainerState);

                    break;


            }
            telemetry.addData("Target Position", slideTrainer.targetHeight);
            telemetry.addData("Actual Position","%.1f", slideTrainer.getSlidePos());
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

    ////////////////////////////  Slide State Switch Case     ////////////////



}
