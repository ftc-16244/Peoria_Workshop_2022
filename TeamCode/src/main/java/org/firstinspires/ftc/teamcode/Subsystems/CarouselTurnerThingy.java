package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.Alliance;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;

public class CarouselTurnerThingy {

    //Define Hardware Objects
    public DcMotor carousel = null;

    public static final double      CAROUSELOFF = 0;
    public static final double      CAROUSELON = 0.40;
    public static final double      CAROUSELONAUTOONLY = 0.25;

    // create a constructor to add in the special requirements



    public void init(HardwareMap hwMap)  {
        carousel = hwMap.get(DcMotor.class,"carousel");
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void carouselTurnOff() {
        carousel.setPower(CAROUSELOFF);
    }
    // CCW is for the blue side
    public void carouselTurnCCW() {
        carousel.setPower(CAROUSELON);
    }

    public void carouselTurnCW() {
        carousel.setPower(-CAROUSELON);
    }

    public void carouselTurnCCWAuto() {
        carousel.setPower(CAROUSELONAUTOONLY);
    }

    public void carouselTurnCWAuto() {
        carousel.setPower(-CAROUSELONAUTOONLY);
    }

    }



