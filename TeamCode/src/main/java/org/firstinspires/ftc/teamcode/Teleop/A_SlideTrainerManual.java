package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Enums.SlideTrainerState;
import org.firstinspires.ftc.teamcode.Subsystems.Slide_Trainer;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

@Config
@TeleOp(group = "Viper Slide Test")
//@Disabled
public class A_SlideTrainerManual<slideTrainerState> extends LinearOpMode {

    FtcDashboard dashboard;

    Slide_Trainer slideTrainer = new Slide_Trainer(this);
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
        float slidePower = 0;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////

        // switch out of encoder mode for this opmode
        slideTrainer.slidemotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {


            // Gampepad 1 Functions
            random_power = -gamepad1.left_stick_y;
            slideTrainer.randomMotor.setPower(random_power);

            slidePower = -gamepad1.right_stick_y;
            slideTrainer.slidemotor.setPower(slidePower);

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

            }

            if (gamepad1.b) {



            }

            if (gamepad1.x) {



            }

            if (gamepad1.y) {


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

               //slideTrainer.slideMechanicalReset();
               //slideTrainer.targetHeight = 0;
               slideTrainerState =  SlideTrainerState.MECH_RESET;
            }

            /**
             *
             * Gamepad #1 DPAD
             *
             **/
            if (gamepad1.dpad_left) {
                slideTrainer.servo.setPosition(0.0);
                telemetry.addData("Servo Position", "0");
            }

            if (gamepad1.dpad_up) {

                slideTrainer.servo.setPosition(0.25);
                telemetry.addData("Servo Position", "25%");
            }

            if (gamepad1.dpad_right) {

                slideTrainer.servo.setPosition(0.5);
                telemetry.addData("Servo Position", "50%");
            }

            if (gamepad1.dpad_down) {

                slideTrainer.servo.setPosition(0.75);
                telemetry.addData("Servo Position", "75%");
            }

            /**
             *
             * Gamepad #1 Triggers
             *
             **/

            if (gamepad1.left_trigger > 0.25) {

                //debounce(400);


                //debounce(400);
            }
            if (gamepad1.right_trigger > 0.25) {


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
            telemetry.addData("Slide Motor Power","%.1f", slideTrainer.slidemotor.getPower());
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
