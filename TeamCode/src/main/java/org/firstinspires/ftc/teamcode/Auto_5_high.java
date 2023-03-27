package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.AprilTagAutonomousInitDetectionExample.FEET_PER_METER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Vector;

@Autonomous
public class Auto_5_high extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotNW Robot = new RobotNW();
        ElapsedTime timer = new ElapsedTime();
        elevatorNewThreadMiddle new_thread = new elevatorNewThreadMiddle();
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;
        Robot.init(hardwareMap, telemetry, this);

        telemetry.addData("", "Init complited");
        telemetry.update();
        final double FEET_PER_METER = 3.28084;
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;
        double y = 6;
        double x = -40;
        // UNITS ARE METERSa
        double tagsize = 0.166;

        int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
        int LEFT = 1;
        int MIDDLE = 12;
        int RIGHT = 3;
        AprilTagDetection tagOfInterest = null;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            //@Override
            public void onError(int errorCode) {

            }
        });

        Pose2d StartPose = new Pose2d(-66, 36, Math.toRadians(0));
        new_thread.HM = hardwareMap;
        new_thread.init();

        Trajectory myTrajectorySpline = drive.trajectoryBuilder(StartPose)
                .lineToSplineHeading(new Pose2d(-40, 42, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-8, 37.0, Math.toRadians(-45)), Math.toRadians(-45))
                .build();
        Trajectory myTrajectorySpline2 = drive.trajectoryBuilder(myTrajectorySpline.end())
                .splineToSplineHeading(new Pose2d(-14, 60, Math.toRadians(-90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-14, 67, Math.toRadians(-90)))
                .build(); //стопка
        Trajectory myTrajectorySpline3 = drive.trajectoryBuilder(myTrajectorySpline2.end())
                .lineToSplineHeading(new Pose2d(-13, 60, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(-10, 48, Math.toRadians(-45)), Math.toRadians(-45))
                .build(); //2 дж
        Trajectory myTrajectorySpline4 = drive.trajectoryBuilder(myTrajectorySpline3.end())
                .splineToSplineHeading(new Pose2d(-13, 70, Math.toRadians(-90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-13, 77, Math.toRadians(-90)))
                .build(); //стопка
        Trajectory myTrajectorySpline5 = drive.trajectoryBuilder(myTrajectorySpline4.end())
                .lineToSplineHeading(new Pose2d(-21, 70, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(-18, 49, Math.toRadians(-45)), Math.toRadians(-45))
                .build(); //3 дж
        Trajectory myTrajectorySpline6 = drive.trajectoryBuilder(myTrajectorySpline5.end())
                .splineToSplineHeading(new Pose2d(-25, 65, Math.toRadians(-90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-25, 90, Math.toRadians(-90)))
                .build(); //стопка
        /*
        Trajectory myTrajectorySplineNew = drive.trajectoryBuilder(myTrajectorySpline.end())
                .lineTo(new Vector2d(-22, 30.2))
                .build();

        Trajectory myTrajectorySplineNew1 = drive.trajectoryBuilder(myTrajectorySpline3.end())
                .forward(1.5)
                .build();

        Trajectory myTrajectorySpline6 = drive.trajectoryBuilder(myTrajectorySplineNew1.end())
                .lineToSplineHeading(new Pose2d(-19, 40, Math.toRadians(90)))
                .build();*/

        /*Trajectory myTrajectorySpline10 = drive.trajectoryBuilder(myTrajectorySpline.end())
                //.lineToSplineHeading(new Pose2d(-10, 60, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-15, 50, Math.toRadians(-90)))
                //.splineToSplineHeading(new Pose2d(-15, 48, Math.toRadians(-135)), Math.toRadians(-135))
                .build(); //2 дж*/

        drive.setPoseEstimate(StartPose);

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            } else {
                telemetry.addLine("Don't see tag of interest :(");
                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            y = 92;
            telemetry.addLine("FIRST");
            telemetry.update();
        } else if (tagOfInterest.id == MIDDLE) {
            y = 61;
            telemetry.addLine("SECOND");
            telemetry.update();
        } else {
            y = 39;
            telemetry.addLine("THIRD");
            telemetry.update();

        }
        Trajectory myTrajectorySpline61 = drive.trajectoryBuilder(myTrajectorySpline6.end())
                .lineToSplineHeading(new Pose2d(-25, y, Math.toRadians(-90)))
                .build();

        waitForStart();
        new_thread.start();
        new_thread.setTargetAng(2080);
        new_thread.EL.changeover.setPosition(0.96);
        drive.followTrajectory(myTrajectorySpline);
        telemetry.update();

        Robot.servo.setPosition(0.8);

        timer.reset();
        while (timer.milliseconds() < 200 && !isStopRequested());

        new_thread.change_per(200);

        new_thread.setTagetLiftAng_timer(800, 300);

        drive.followTrajectory(myTrajectorySpline2);
        new_thread.setTargetAng(270);
        timer.reset();
        while (timer.milliseconds() < 200 && !isStopRequested()) ;
        Robot.servo.setPosition(1);


        timer.reset();
        while (timer.milliseconds() < 200 && !isStopRequested());
        new_thread.change_per(0);
        new_thread.setTargetAng(2080);
        drive.followTrajectory(myTrajectorySpline3);
        Robot.servo.setPosition(0.8);

        timer.reset();
        while (timer.milliseconds() < 200 && !isStopRequested());

        new_thread.change_per(200);

        new_thread.setTagetLiftAng_timer(750, 300);

        drive.followTrajectory(myTrajectorySpline4);
        new_thread.setTargetAng(220);
        timer.reset();
        while (timer.milliseconds() < 200 && !isStopRequested()) ;
        Robot.servo.setPosition(1);

        /*timer.reset();
        while (timer.milliseconds() < 200 && !isStopRequested());
        new_thread.change_per(0);
        new_thread.setTargetAng(2080);
        drive.followTrajectory(myTrajectorySpline5);
        Robot.servo.setPosition(0.8);

        timer.reset();
        while (timer.milliseconds() < 200 && !isStopRequested());

        new_thread.change_per(200);

        new_thread.setTagetLiftAng_timer(700, 300);

        drive.followTrajectory(myTrajectorySpline6);
        new_thread.setTargetAng(170);
        timer.reset();
        while (timer.milliseconds() < 200 && !isStopRequested()) ;
        Robot.servo.setPosition(1);
        timer.reset();
        while (timer.milliseconds() < 200 && !isStopRequested()) ;

        new_thread.setTargetAng(800);
        new_thread.setTagetLiftAng_timer(0, 500);
        drive.followTrajectory(myTrajectorySpline61);*/

        new_thread.interrupt();
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}


