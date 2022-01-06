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
public class RedWarehouseMeet3 extends LinearOpMode {

    OpenCvCamera webcam;

    public static double DISTANCE = 30; // in
    FelipeDeux felipe = new FelipeDeux(this); // instantiate Felipe (the main implement)
    CarouselTurnerThingy carousel = new CarouselTurnerThingy();
    // init and setup
    ElapsedTime runtime = new ElapsedTime();
    Barcode barcode = Barcode.RIGHT; // Default target zone


    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FreightFrenzyTSEPipeline_EXP detector = new FreightFrenzyTSEPipeline_EXP(telemetry, Alliance.RED, StartSide.WAREHOUSE);
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
        double delayTime = 10; // delay time
        // initialize the other subsystems
        felipe.init(hardwareMap);
        carousel.init(hardwareMap);
        felipe.juanMechanicalReset();


        //for high goal
        Trajectory  traj1high = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(43,2,Math.toRadians(179)))
                .addTemporalMarker(-.25,()->{felipe.armMid();})
                .build();
        Trajectory  traj2high = drive.trajectoryBuilder(traj1high.end())
                .addTemporalMarker(-0.6,()->{felipe.thumbOpen();})
                .addTemporalMarker(.5,()->{felipe.thumbClose();})
                .addTemporalMarker(.7,()->{felipe.armInit();})
                .strafeLeft(4)
                .build();
        Trajectory  traj3high = drive.trajectoryBuilder(traj2high.end())
                .lineToLinearHeading(new Pose2d(-4,-2,Math.toRadians(270)))
                .build();
        Trajectory  traj4high = drive.trajectoryBuilder(traj3high.end())
                .forward(30)
                .build();
        Trajectory  traj5high = drive.trajectoryBuilder(traj4high.end())
                .strafeLeft(25)
                .build();

        //for middle goal
        Trajectory  traj1mid = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(26,15,Math.toRadians(90)))
                .addTemporalMarker(-.25,()->{felipe.armMid();})
                .build();
        Trajectory  traj2mid = drive.trajectoryBuilder(traj1mid.end())
                .strafeLeft(3)
                .addTemporalMarker(-0.6,()->{felipe.thumbOpen();})
                .addTemporalMarker(.5,()->{felipe.thumbClose();})
                .addTemporalMarker(.7,()->{felipe.armInit();})
                .build();
        Trajectory  traj3mid = drive.trajectoryBuilder(traj2mid.end())
                .lineToLinearHeading(new Pose2d(17,12,Math.toRadians(-90)))
                .build();
        Trajectory  traj4mid = drive.trajectoryBuilder(traj3mid.end())
                .lineToLinearHeading(new Pose2d(-4,-2,Math.toRadians(-90)))
                .build();
        Trajectory  traj5mid = drive.trajectoryBuilder(traj4mid.end())
                .forward(29)
                .build();
        Trajectory  traj6mid = drive.trajectoryBuilder(traj5mid.end())
                .strafeLeft(25)
                .build();

        //for low goal
        Trajectory  traj1low = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(26,15,Math.toRadians(90)))
                .addTemporalMarker(-.25,()->{felipe.armLow();})
                .build();
        Trajectory  traj2low = drive.trajectoryBuilder(traj1low.end())
                .strafeLeft(6)
                .addTemporalMarker(-0.6,()->{felipe.thumbOpen();})
                .addTemporalMarker(2,()->{felipe.thumbClose();})
                .addTemporalMarker(.7,()->{felipe.armInit();})
                .build();
        Trajectory  traj3low = drive.trajectoryBuilder(traj2low.end())
                .lineToLinearHeading(new Pose2d(17,12,Math.toRadians(-90)))
                .build();
        Trajectory  traj4low = drive.trajectoryBuilder(traj3low.end())
                .lineToLinearHeading(new Pose2d(-4,-2,Math.toRadians(-90)))
                .build();
        Trajectory  traj5low = drive.trajectoryBuilder(traj4low.end())
                .forward(29)
                .build();
        Trajectory  traj6low = drive.trajectoryBuilder(traj5low.end())
                .strafeLeft(25)
                .build();




        waitForStart();
        felipe.liftLoad();// put here becase opmode is acitve is a condition in the method that does this

            switch(detector.getLocation()){
                case LEFT: //
                    timer.reset();
                    while(timer.seconds() < delayTime) drive.update();
                    drive.followTrajectory(traj1low);
                    drive.followTrajectory(traj2low);
                    drive.followTrajectory(traj3low);
                    drive.followTrajectory(traj4low);
                    drive.followTrajectory(traj5low);
                    drive.followTrajectory(traj6low);
                    felipe.liftLoad();

                    break;

                case CENTER: //
                    timer.reset();
                    while(timer.seconds() < delayTime) drive.update();
                    drive.followTrajectory(traj1mid);
                    drive.followTrajectory(traj2mid);
                    drive.followTrajectory(traj3mid);
                    drive.followTrajectory(traj4mid);
                    drive.followTrajectory(traj5mid);
                    drive.followTrajectory(traj6mid);
                    felipe.liftLoad();

                    break;

                case RIGHT: //level 3 highest goal

                    felipe.liftRise();
                    drive.followTrajectory(traj1high);
                    drive.followTrajectory(traj2high);
                    drive.followTrajectory(traj3high);
                    drive.followTrajectory(traj4high);
                    drive.followTrajectory(traj5high);
                    felipe.liftLoad();

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
