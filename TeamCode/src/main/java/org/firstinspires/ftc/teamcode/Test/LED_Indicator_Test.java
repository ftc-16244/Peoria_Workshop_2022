package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class LED_Indicator_Test extends LinearOpMode {
    private TouchSensor touch;
    private DigitalChannel redLED;
    private DigitalChannel greenLED;


    @Override
    public void runOpMode() {
        // Get the LED colors and touch sensor from the hardwaremap
        //redLED = hardwareMap.get(DigitalChannel.class, "red");
        //greenLED = hardwareMap.get(DigitalChannel.class, "green");
        touch = hardwareMap.get(TouchSensor.class, "touch");

        // Wait for the play button to be pressed
        waitForStart();

        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            if (!touch.isPressed()){
                //Touch Sensor is not pressed
                greenLED.setState(true);
                redLED.setState(false);

            } else {
                //Touch Sensor is pressed
                redLED.setState(true);
                greenLED.setState(false);
            }

        }
    }
}