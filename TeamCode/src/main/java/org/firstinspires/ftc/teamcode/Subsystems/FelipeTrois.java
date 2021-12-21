package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.LiftPosition;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static java.lang.Thread.sleep;

public class FelipeTrois {

    //Define Hardware Objects


    public DcMotorEx    linearActuator   = null;
    public DcMotor      patrickIntake    = null; // no longer used
    public DcMotorEx    julioArm         = null; // new
    public Servo        homieBox         = null;
    public Servo        cristianoCodo    = null; //arm
    public Servo        panchoPulgar     = null; //thumb

    // Need some features from the Linear Opmode to make the lift work


    ElapsedTime runtime = new ElapsedTime();

    //Constants Lift
    public static final double      JUANLIFTSPEED               =   1.0; //
    //public static final int       JUANLIFTDOWN                =   0; // use the LOAD instead of down. Zero pushes wheels off the mat
    public static final double      JUANLIFTPARTIAL             =   5.25;
    public static final int         JUANLIFTLOW                 =   2;
    public static final double      JUANLIFTUP                  =   7.0; //Number is in inches
    public static final double      JUANLIFTLOAD                =   0.5; //Number is in inches
    private static final double     TICKS_PER_MOTOR_REV         =   145.1; // goBilda 1150 RPM motor
    private static final double     ACTUATOR_DISTANCE_PER_REV   = 8/25.4; // 8mm of travel per rev converted to inches
    public static final double      TICKS_PER_LIFT_IN           = TICKS_PER_MOTOR_REV / ACTUATOR_DISTANCE_PER_REV; // 460 and change

    public static final double      CRISTIANOCODOINIT =  0.18;
    public static final double      CRISTIANOCODOMID =  0.5;
    public static final double      CRISTIANOCODOLOW =  0.7;

    public static final double      PANCHOPULGAROPEN =  0.2;
    public static final double      PANCHOPULGARCLOSE =  0.525;


    //Constants for new motor version of Julio
    public static final double      JULIOARMLEFT            =   -100.0;
    public static final double      JULIOARMLEFT45          =   -65.0;
    public static final double      JULIOARMCENTER          =   0.0;
    public static final double      JULIOARMRIGHT           =   100.0;
    public static final double      JULIOARMRIGHT45          =  65.0;
    public static final double      JULIOTURNSPEED          =   0.5; // if this goes to fast it bounces back and hits the frame
    public static final double      TICKS_PER_REV           =   1425.1; // 117 RPM motor 50.9:1 reduction
    public static final double      TICKS_PER_DEGREE         =  TICKS_PER_REV/360;

    //Constants for robot home box
    public static final double      HOMIEBOXPIVOTLEFT       = 1;
    public static final double      HOMIEBOXPIVOTRIGHT      = 0.3;
    public static final double      HOMIEBOXPIVOTCENTER     = 0.66;
    public static final double      HOMIEDELAY              = 0.15;

    //Constants for robot intake
    public static final double      PATRICKINTAKESLOW = .3;//use this while lifting juan
    public static final double      PATRIKINTAKECOFF = 0;
    public static final double      PATRICKINTAKEON = 0.7;


    public static final double JULIO_NEW_P = 10; // 2.5
    public static final double JULIO_NEW_I = 0.5;// 0.1
    public static final double JULIO_NEW_D = 0.5; // 0.2
    public static final double JULIO_NEW_F = 10; // 10

    public static final double JUAN_NEW_P = 5; // 2.5
    public static final double JUAN_NEW_I = 0.5;// 0.1
    public static final double JUAN_NEW_D = 0.0; // 0.2
    public static final double JUAN_NEW_F = 20; // 10

    PIDFCoefficients pidJULIO_fNew = new PIDFCoefficients(JULIO_NEW_P, JULIO_NEW_I, JULIO_NEW_D, JULIO_NEW_F);
    PIDFCoefficients pidJUAN_fNew = new PIDFCoefficients(JUAN_NEW_P, JUAN_NEW_I, JUAN_NEW_D, JUAN_NEW_F);


    LiftPosition liftPosition = LiftPosition.UNKNOWN;


    LinearOpMode opmode;

    public FelipeTrois(LinearOpMode opmode) {
        this.opmode = opmode;
    }


    public void init(HardwareMap hwMap)  {

        // Initialize Juan - Linear Actuator type of lift
        linearActuator = hwMap.get(DcMotorEx.class,"juanLift");
        linearActuator.setDirection(DcMotor.Direction.FORWARD);
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidJUAN_fNew);

        //Initialize Patrick which is the intake drive motor
        patrickIntake = hwMap.get(DcMotor.class,"patrickIntake");
        patrickIntake.setDirection(DcMotor.Direction.FORWARD);
        patrickIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Julio the rotating arm to deliver Homie to the sides
        julioArm = hwMap.get(DcMotorEx.class,"julioArm");
        julioArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        julioArm.setDirection(DcMotorEx.Direction.FORWARD);
        julioArm.setDirection(DcMotorEx.Direction.FORWARD);
        julioArm.setZeroPowerBehavior(BRAKE);
        julioArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // get default PIDF values
       julioArm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidJULIO_fNew);


        // Initialize Homie the servo that rotates the freight capture bucket
        homieBox = hwMap.get(Servo.class,"homieBox");

        // Initialize the aux arm for preloaded box
        cristianoCodo = hwMap.get(Servo.class,"arm");

        // Initialize the aux thumb for preloaded box
        panchoPulgar = hwMap.get(Servo.class,"thumb");

        // pre-position servos
        homieBox.setPosition(HOMIEBOXPIVOTCENTER);
        panchoPulgar.setPosition(PANCHOPULGARCLOSE);
        cristianoCodo.setPosition(CRISTIANOCODOINIT);

        // get motor detaisl


    }

       //Juan the lift's methods -  while loop type (use during Auto)

    public void liftRise() {
        liftToTargetHeight(JUANLIFTUP ,3);
    }
    public void liftPartial() {
        liftToTargetHeight(JUANLIFTPARTIAL ,3);
    }
    public void liftLoad() {
        liftToTargetHeight(JUANLIFTLOAD,1);
        liftPosition = LiftPosition.LOAD; // set state accordingly after this is done
    }

    // get Juan's Position need a local variable to do this
    public double getJuanPosition(){
        double juanPositionLocal;
        juanPositionLocal = linearActuator.getCurrentPosition()/ TICKS_PER_LIFT_IN; //returns in inches
        return  juanPositionLocal;
    }

    public void setJuanToLoad(){
        linearActuator.setTargetPosition( (int)(JUANLIFTLOAD *  TICKS_PER_LIFT_IN));

    }

    public void setJuanToPartial(){
        linearActuator.setTargetPosition( (int)(JUANLIFTPARTIAL *  TICKS_PER_LIFT_IN));

    }

    // Juan mechanical reset use in all opmodes Telop and Auto to reset the encoders

    public void juanMechanicalReset(){
        linearActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // need to swich off encoder to run with a timer
        linearActuator.setPower(-0.6);
        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        while ((runtime.seconds() < 2.0)) {

            //Time wasting loop
        }
        // set everything back the way is was before reset so encoders can be used
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftPosition = LiftPosition.MECH_RESET; // we know the lift is at zero here.

    }


    //Julio the arm's methods
    public double getJulioPosition(){
        double julioPositionLocal;
        julioPositionLocal = julioArm.getCurrentPosition()/  TICKS_PER_DEGREE; //returns in degrees
        return  julioPositionLocal;
    }


    public void setJulioToZero(){
        julioArm.setTargetPosition(0);

    }

    public void setJulioTo90Right(){
        julioArm.setTargetPosition( (int)(JULIOARMRIGHT *  TICKS_PER_DEGREE));

    }

    public void setJulioTo45Right(){
        julioArm.setTargetPosition( (int)(JULIOARMRIGHT45 *  TICKS_PER_DEGREE));

    }

    public void setJulioTo90Left(){
        julioArm.setTargetPosition( (int)(JULIOARMLEFT *  TICKS_PER_DEGREE));

    }

    public void setJulioTo45Left(){
        julioArm.setTargetPosition( (int)(JULIOARMLEFT45 *  TICKS_PER_DEGREE));

    }

    public void julioLeft90(){
        rotateToTargetAngle(JULIOARMLEFT,2);
    }

    public void julioRight90(){
        rotateToTargetAngle(JULIOARMRIGHT  ,2);
    }

    public void julioCenter(){
        rotateToTargetAngle(JULIOARMCENTER  ,2);
        julioArm.setPower(Math.abs(0)); // cut power to motor when it is at the center to it can be guided into the frame
    }

    // Intake and Homie box methods used in teleop and auto they are simple

    //Patrick the intake's methods
    public void intakeOff() {
        patrickIntake.setPower(PATRIKINTAKECOFF);
    }
    public void intakeOn() {
        patrickIntake.setPower(PATRICKINTAKEON);
    }
    public void intakeEject() {
        patrickIntake.setPower(-PATRICKINTAKEON);
    }

    //Homie the box's methods
    public void hoimeDumpLeft() throws InterruptedException {
        homieRight();
        sleep(500);
        homieCenter();
        sleep(500);

    }

    public void hoimeDumpRight() throws InterruptedException {
        homieLeft();
        sleep(500);
        homieCenter();
        sleep(500);

    }

    public void homieRight() {
        homieBox.setPosition(HOMIEBOXPIVOTRIGHT);
    }
    public void homieLeft() {
        homieBox.setPosition(HOMIEBOXPIVOTLEFT);
    }
    public void homieCenter() {
        homieBox.setPosition(HOMIEBOXPIVOTCENTER);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Auto Only for dropping the preloaded box - no Teleop controls necessary (except in a test file)
    public void armInit(){
        cristianoCodo.setPosition(CRISTIANOCODOINIT);
    }

    public void armMid(){
        cristianoCodo.setPosition(CRISTIANOCODOMID);
    }

    public void armLow(){
        cristianoCodo.setPosition(CRISTIANOCODOLOW);
    }
    public void thumbOpen(){
        panchoPulgar.setPosition((PANCHOPULGAROPEN));
    }
    public void thumbClose(){
        panchoPulgar.setPosition((PANCHOPULGARCLOSE));
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // While loop methods for the arm and lift - DO NOT USE IN TELEOP or Driving will be imparred
    public void rotateToTargetAngle(double angle, double timeoutS){

        int newTargetAngle; // ok to leave an an int unless we want really fine tuning


        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetAngle = (int)(angle *  TICKS_PER_DEGREE);
            // Set the target now that is has been calculated
            julioArm.setTargetPosition(newTargetAngle); //
            // Turn On RUN_TO_POSITION
            julioArm.setPower(Math.abs( JULIOTURNSPEED ));
            // reset the timeout time and start motion.
            runtime.reset();
            julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and there is time left, and thr motor is running.
            // Note: We use (isBusy() in the loop test, which means that when the motor hits
            // its target position, motion will stop.

            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && julioArm.isBusy()) {

                // Display it for the driver.
                //  telemetry.addData("Moving to New Lift Height",  "Running to %7d", newTargetHeight);

                // telemetry.update();
            }

        }
    }

    public void liftToTargetHeight(double height, double timeoutS){

        int newTargetHeight;


        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetHeight = (int)(height *  TICKS_PER_LIFT_IN);
            // Set the target now that is has been calculated
            linearActuator.setTargetPosition(newTargetHeight); //1000 ticks extends lift from 295mm to 530 mm which is 9.25 in per 1000 ticks or 108 ticks per in
            // Turn On RUN_TO_POSITION
            linearActuator.setPower(Math.abs(JUANLIFTSPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and there is time left, and thr motor is running.
            // Note: We use (isBusy() in the loop test, which means that when the motor hits
            // its target position, motion will stop.

            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && linearActuator.isBusy()) {

                // Display it for the driver.
                //  telemetry.addData("Moving to New Lift Height",  "Running to %7d", newTargetHeight);

                // telemetry.update();
            }

        }
    }



}




