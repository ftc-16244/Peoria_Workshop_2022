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
public class NikitaRedCarouselAutoMeet3 extends LinearOpMode {

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
        FreightFrenzyTSEPipeline_EXP detector = new FreightFrenzyTSEPipeline_EXP(telemetry, Alliance.RED, StartSide.CAROUSEL);
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
        // Trajectories - HIGH GOAL
        ///////////////////////////////////////////////////////////////////////////
        Trajectory  traj0 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(39,30,Math.toRadians(0)))
                //.addTemporalMarker(-.25,()->{felipe.armMid();})
                //.addTemporalMarker(-.25,()->{felipe.liftRise();})
                .build();
        Trajectory  traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(37,-5,Math.toRadians(0)))
                .addTemporalMarker(.1,()->{felipe.armMid();})
                //.addTemporalMarker(-.25,()->{felipe.liftRise();})
                .build();

        Trajectory  traj2 = drive.trajectoryBuilder(traj1.end())
                .addTemporalMarker(-0.8,()->{felipe.thumbOpen();})
                .addTemporalMarker(1,()->{felipe.thumbClose();})
                .addTemporalMarker(1.5,()->{felipe.armInit();})
                .lineToLinearHeading(new Pose2d(39,26,Math.toRadians(90)))

                .build();

        Trajectory  traj2A = drive.trajectoryBuilder(traj2.end())

                .lineToLinearHeading(new Pose2d(11,28,Math.toRadians(90)))

                .build();

        Trajectory  traj3 = drive.trajectoryBuilder(traj2A.end())
                // final touch up to engage carousel
                .strafeLeft(7)
                .addTemporalMarker(1,()->{carousel.carouselTurnCWAuto();})
                .build();
        Trajectory  traj4 = drive.trajectoryBuilder(traj3.end())
                //back away but stay out of the wall to make it move better
                .lineToLinearHeading(new Pose2d(30,29,Math.toRadians(-90)))
                .addTemporalMarker(.25,()->{carousel.carouselTurnOff();})
                .build();
        Trajectory  traj5 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(41,3,Math.toRadians(179)))
                .addTemporalMarker(-.25,()->{felipe.armLow();})
                //.addTemporalMarker(-.25,()->{felipe.liftRise();})
                .build();
        Trajectory  traj6 = drive.trajectoryBuilder(traj5.end())
                .addTemporalMarker(-0.6,()->{felipe.thumbOpen();})
                .addTemporalMarker(.1,()->{felipe.thumbClose();})
                .addTemporalMarker(.5,()->{felipe.armInit();})
                .lineToLinearHeading(new Pose2d(9,-28,Math.toRadians(-180)))

                .build();
        Trajectory  traj7 = drive.trajectoryBuilder(traj6.end())
                // final touch up to engage carousel
                .forward(5)
                .addTemporalMarker(1,()->{carousel.carouselTurnCCW();})
                .build();
        Trajectory  traj8 = drive.trajectoryBuilder(traj7.end())
                //back away but stay out of the wall to make it move better
                .lineToLinearHeading(new Pose2d(26,-28,Math.toRadians(90)))
                .addTemporalMarker(.25,()->{carousel.carouselTurnOff();})
                .build();

        Trajectory  traj5low = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(37,-2,Math.toRadians(0)))
                .addTemporalMarker(-.25,()->{felipe.armLow();})
                .build();

        Trajectory  traj6low = drive.trajectoryBuilder(traj5low.end())
                .addTemporalMarker(-0.8,()->{felipe.thumbOpen();})
                .addTemporalMarker(1,()->{felipe.thumbClose();})
                .addTemporalMarker(1.5,()->{felipe.armInit();})
                .lineToLinearHeading(new Pose2d(39,29,Math.toRadians(90)))

                .build();
        Trajectory  traj7low = drive.trajectoryBuilder(traj6low.end())

                .lineToLinearHeading(new Pose2d(11,29,Math.toRadians(90)))

                .build();

        Trajectory  traj8low = drive.trajectoryBuilder(traj7low.end())
                // final touch up to engage carousel
                .strafeLeft(7)
                .addTemporalMarker(1,()->{carousel.carouselTurnCWAuto();})
                .build();
        Trajectory  traj9low = drive.trajectoryBuilder(traj8low.end())
                //back away but stay out of the wall to make it move better
                .lineToLinearHeading(new Pose2d(30,29,Math.toRadians(-90)))
                .addTemporalMarker(.25,()->{carousel.carouselTurnOff();})
                .build();
        waitForStart();
        felipe.liftLoad();// put here becase opmode is acitve is a condition in the method that does this
        tfTime.reset(); //  reset the TF timer

            switch(detector.getLocation()){
                case LEFT: //
                    drive.followTrajectory(traj5low);
                    drive.followTrajectory(traj6low);
                    felipe.liftLoad();
                    drive.followTrajectory(traj7low);
                    drive.followTrajectory(traj8low);


                    //delay to let carousel turn
                    timer.reset();
                    while(timer.seconds() < ducktime) drive.update();

                    drive.followTrajectory(traj9low);

                    break;

                case CENTER: //
                    drive.followTrajectory(traj0);
                    drive.followTrajectory(traj1);
                    drive.followTrajectory(traj2);
                    drive.followTrajectory(traj2A);
                    drive.followTrajectory(traj3);

                    //delay to let carousel turn
                    timer.reset();
                    while(timer.seconds() < ducktime) drive.update();

                    drive.followTrajectory(traj4);


                    break;

                case RIGHT: //level 3 highest goal
                    felipe.liftRise();
                    drive.followTrajectory(traj0);
                    drive.followTrajectory(traj1);
                    drive.followTrajectory(traj2);
                    drive.followTrajectory(traj2A);
                    felipe.liftLoad();
                    drive.followTrajectory(traj3);

                    //delay to let carousel turn
                    timer.reset();
                    while(timer.seconds() < ducktime) drive.update();

                    drive.followTrajectory(traj4);
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
