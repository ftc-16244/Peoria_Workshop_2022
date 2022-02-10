package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.FelipeTrois;
@TeleOp(group = "Test")
@Disabled

public class PidTester extends LinearOpMode {


    // our DC motor.
    FelipeTrois felipe = new FelipeTrois(this);

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 10;
    public int tol;
    public int newtolerance = 3;

    public void runOpMode() {
        // get reference to DC motor.
        // the class that this coms from already has it set up as a DcMotorEx
        // Find the default PIDF values inorder to make intelligent modifications.

        // adapted from here
        // https://github.com/ftctechnh/ftc_app/wiki/Changing-PID-Coefficients


        // wait for start command.
        waitForStart();

        felipe.init(hardwareMap);

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidfOrig = felipe.julioArm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        felipe.julioArm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = felipe.julioArm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        tol = felipe.julioArm.getTargetPositionTolerance();
        felipe.julioArm.setTargetPositionTolerance(newtolerance);

        // display info to user.
        while(opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                    pidModified.p, pidModified.i, pidModified.d, pidModified.f);
            telemetry.addData("Encoder Tolerance",  tol);
            telemetry.update();
        }
    }
}
