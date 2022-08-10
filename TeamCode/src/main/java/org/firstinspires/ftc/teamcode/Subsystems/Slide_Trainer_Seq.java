package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.SlideTrainerState;

@Config // this is so the dashboard will pick up variables
public class Slide_Trainer_Seq {

    //Define Hardware Objects


    public  DcMotorEx       slidemotor;
    public  DcMotor         randomMotor = null;
    private RevTouchSensor  lowerLimitSwitch        = null; // always an odd port
    public  DigitalChannel  redLED                  = null; // this goes on an odd port of Digital Channels 1,3,5,7
    public  DigitalChannel  greenLED                = null; // green connects to an even port 2,4,6,8
    public  Servo           servo;

    Telemetry       telemetry;
    LinearOpMode    opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();

    //Constants Lift
    public  static double            SLIDELIFTSPEED                  = 1.0; //
    public static  double            SLIDELOWERSPEED                 = -0.4; // use the LOAD instead of down. Zero pushes wheels off the mat
    private static final double      SLIDE_LEVEL_1                   = 1; // inches
    private static final double      SLIDE_LEVEL_2                   = 4; // inches
    private static final double      SLIDE_LEVEL_3                   = 7; // inches
    private static final double      SLIDE_LEVEL_4                   = 10; // inches


    private static final double     TICKS_PER_MOTOR_REV             = 537.7; // goBilda 312 RPM
    private static final double     PULLEY_DIA                      = 40; // milimeters
    private static final double     SLIDE_LIFT_DISTANCE_PER_REV     = PULLEY_DIA * Math.PI / 25.4; //  lift = circimference of the pulley converted to inches
    private static final double      TICKS_PER_LIFT_IN               = 109; //TICKS_PER_MOTOR_REV /  SLIDE_LIFT_DISTANCE_PER_REV; // 109 and change

    public static double SLIDE_NEW_P = 10.0; // 2.5 default
    public static double SLIDE_NEW_I = 0.1;// 0.1 default
    public static double SLIDE_NEW_D = 0.0; // 0.2 default
    public static double SLIDE_NEW_F = 0; // 10 default


    public double  targetHeight;


    int tol;
    int newtolerance;





   SlideTrainerState slideTrainerState = SlideTrainerState.UNKNOWN;



    //Telemetry telemetry;

// constructor with opmmode passed in
    public Slide_Trainer_Seq(LinearOpMode opmode) {
       this.opmode = opmode;

   }

   public void init(HardwareMap hwMap)  {


        // Initialize the random motor
        randomMotor = hwMap.get(DcMotor.class, "random_motor");
        randomMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize the slide motor
        slidemotor = hwMap.get(DcMotorEx.class,"slideMotor");
        slidemotor.setDirection(DcMotorEx.Direction.REVERSE);

        // servo
        servo = hwMap.get(Servo.class, "servo");
        servo.setPosition(0.0);

        //Initialize the LED and change to output
        redLED = hwMap.get(DigitalChannel.class, "red"); // Digital Device - Digital Device (not LED)
        greenLED = hwMap.get(DigitalChannel.class, "green");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        //Initialize the limit switch
        lowerLimitSwitch = hwMap.get(RevTouchSensor.class, "limitswitch"); // Digital Device - REV Touch Sensor (not Analog MR Touch)


        // set LED to red to show that we need to initialize
        redLED.setState(true);
        greenLED.setState(false);

        PIDFCoefficients pidfOrig = slidemotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidSlide_New = new PIDFCoefficients(SLIDE_NEW_P, SLIDE_NEW_I, SLIDE_NEW_D, SLIDE_NEW_F);
        slidemotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSlide_New);
        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = slidemotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //slideTrainer.setSlideLow(); comment out to leave in IDLE state if desired.




        //tol = slidemotor.getTargetPositionTolerance();
        //slidemotor.setTargetPositionTolerance(newtolerance);


    }





    ///////////////////////////////////////////////////////////////////////////////////////////////
    // simple getter and setter methods
    public double getSlidePos(){
        double slidePos;
        slidePos = slidemotor.getCurrentPosition()/ TICKS_PER_LIFT_IN; //returns in inches
        return  slidePos;
    }



    public void  setSlideLevel1(){

        targetHeight = ( SLIDE_LEVEL_1 );
        liftToTargetHeight(targetHeight,3);
        servo.setPosition(0);

    }

    public void setSlideLevel2(){
        targetHeight = ( SLIDE_LEVEL_2);
        liftToTargetHeight(targetHeight,3);
        servo.setPosition(0);


    }

    public void setSlideLevel3(){
        targetHeight = ( SLIDE_LEVEL_3);
        liftToTargetHeight(targetHeight,3);
        servo.setPosition(1.0);


    }

    public void setSlideLevel4(){
        targetHeight = ( SLIDE_LEVEL_4);
        liftToTargetHeight(targetHeight,3);
        servo.setPosition(1.0);


    }


    public void slideMechanicalReset(){

        slidemotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // need to swich off encoder to run with a timer
        slidemotor.setPower(SLIDELOWERSPEED);
        servo.setPosition(0);
        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        // reset for time allowed or until the limit/ touch sensor is pressed.
        while (runtime.seconds() < 3.0 && !lowerLimitSwitch.isPressed()) {


            //Time wasting loop so slide can retract. Loop ends when time expires or tiuch sensor is pressed
        }
        slidemotor.setPower(0);
        runtime.reset();
        while ((runtime.seconds() < 0.25)) {

            //Time wasting loop to let spring relax
        }
        // set everything back the way is was before reset so encoders can be used
       slidemotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
       slidemotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
       //slideTrainerState = SlideTrainerState.IDLE;// once this is done we are at zero power or idling.

        redLED.setState(false);
        greenLED.setState(true);


    }

    public void liftToTargetHeight(double height, double timeoutS){

        int newTargetHeight;


        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetHeight = (int)(height *  TICKS_PER_LIFT_IN);
            // Set the target now that is has been calculated
            slidemotor.setTargetPosition(newTargetHeight);
            // Turn On RUN_TO_POSITION
            slidemotor.setPower(Math.abs(SLIDELIFTSPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            slidemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and there is time left, and thr motor is running.
            // Note: We use (isBusy() in the loop test, which means that when the motor hits
            // its target position, motion will stop.

            while (opmode.opModeIsActive() &&
                   (runtime.seconds() < timeoutS) && slidemotor.isBusy()) {


            }



        }


    }

}




