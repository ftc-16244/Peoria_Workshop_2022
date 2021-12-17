package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.Barcode;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.FelipeDeux;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Test")
public class NikitaBlueWarehouseMeet3 extends LinearOpMode {

    OpenCvCamera webcam;

    public static double DISTANCE = 30; // in
    public ElapsedTime   tfTime      = new ElapsedTime(); // timer for tensor flow
    FelipeDeux felipe = new FelipeDeux(this); // instantiate Felipe (the main implement)
    CarouselTurnerThingy carousel = new CarouselTurnerThingy();
    // init and setup
    ElapsedTime runtime = new ElapsedTime();
    Barcode barcode = Barcode.RIGHT; // Default target zone

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FreightFrenzyTSEPipeline_EXP detector = new FreightFrenzyTSEPipeline_EXP(telemetry, Alliance.BLUE, StartSide.WAREHOUSE);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //See EaayopenCV webcam example for details on this new streaming methods works
                // manay examples from Skystone and Ultimate goal have outdated methods that are deprecated

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ElapsedTime timer = new ElapsedTime();
        double ducktime = 2.5; // carosel rotation time
        // initialize the other subsystems
        felipe.init(hardwareMap);
        carousel.init(hardwareMap);
        felipe.juanMechanicalReset();

        ///////////////////////////////////////////////////////////////////////////
        Trajectory  traj5 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(39,-4,Math.toRadians(0)))
                .addTemporalMarker(-.25,()->{felipe.armLow();})
                .build();
        Trajectory  traj6 = drive.trajectoryBuilder(traj5.end())
                .addTemporalMarker(-0.6,()->{felipe.thumbOpen();})
                .addTemporalMarker(.1,()->{felipe.thumbClose();})
                .addTemporalMarker(.5,()->{felipe.armInit();})
                .lineToLinearHeading(new Pose2d(-4,2,Math.toRadians(90)))
                .build();

        waitForStart();
        felipe.liftLoad();// put here becase opmode is acitve is a condition in the method that does this
        tfTime.reset(); //  reset the TF timer

            switch(detector.getLocation()){
                case LEFT: //
                    drive.followTrajectory(traj5);
                    drive.followTrajectory(traj6);
                    felipe.liftLoad();
                    break;

                case CENTER: //


                    break;

                case RIGHT: //level 3 highest goal



                    break;
            }
        //}

        if (isStopRequested()) return;


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;

        webcam.stopStreaming();
    }

//checking if my github works
}
