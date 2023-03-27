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
public class park extends LinearOpMode {

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

        //Pose2d StartPose = new Pose2d(-57, -36, Math.toRadians(0));
        new_thread.HM = hardwareMap;
        new_thread.init();

        Trajectory myTrajectorySpline = drive.trajectoryBuilder(new Pose2d())
                .forward(52)
                .build();
        Trajectory myTrajectorySpline2 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(24)
                .build(); //стопка
        Trajectory myTrajectorySpline3 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(24)
                .build();
        /*Trajectory myTrajectorySpline4 = drive.trajectoryBuilder(myTrajectorySpline3.end())
                .splineToSplineHeading(new Pose2d(-12, 65, Math.toRadians(-90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-12, 90, Math.toRadians(-90)))
                .build(); //стопка
        Trajectory myTrajectorySpline5 = drive.trajectoryBuilder(myTrajectorySpline2.end())
                .lineToSplineHeading(new Pose2d(-11, 70.1, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(-22, 52, Math.toRadians(-135)), Math.toRadians(-135))
                .build(); //2 дж
        Trajectory myTrajectorySpline6 = drive.trajectoryBuilder(myTrajectorySpline3.end())
                .splineToSplineHeading(new Pose2d(-11, 65.1, Math.toRadians(-90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-11, 90, Math.toRadians(-90)))
                .build();
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

        //drive.setPoseEstimate(StartPose);

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
            y = 83;
            telemetry.addLine("FIRST");
            telemetry.update();
        } else if (tagOfInterest.id == MIDDLE) {
            y = 53;
            telemetry.addLine("SECOND");
            telemetry.update();
        } else {
            y = 30;
            telemetry.addLine("THIRD");
            telemetry.update();

        }
        Trajectory myTrajectorySpline61 = drive.trajectoryBuilder(myTrajectorySpline.end())
                .lineToSplineHeading(new Pose2d(-13, y, Math.toRadians(-90)))
                .build();

        waitForStart();
        drive.followTrajectory(myTrajectorySpline);
        if (tagOfInterest.id == LEFT){
            drive.followTrajectory(myTrajectorySpline2);
        }
        if (tagOfInterest.id == RIGHT){
            drive.followTrajectory(myTrajectorySpline3);
        }

/*        drive.followTrajectory(myTrajectorySplineNew1);
        Robot.servo.setPosition(0.8);
        timer.reset();
        while (timer.milliseconds() < 500 && !isStopRequested());
        drive.followTrajectory(myTrajectorySpline6);
        new_thread.change_per(500);
        drive.followTrajectory(myTrajectorySpline61);

        timer.reset();
        while (timer.milliseconds() < 1000 && !isStopRequested());
        /*new_thread.launch1 = true;
        drive.followTrajectory(myTrajectorySpline21);
        new_thread.EL.changeover.setPosition(0.89);
        timer.reset();
        while (timer.milliseconds() < 1700 && !isStopRequested()) ;
       Robot.servo.setPosition(1.);
        new_thread.launch2 = true;
        new_thread.change_per(3500);
        timer.reset();
        while (timer.milliseconds() < 3000 && !isStopRequested()) ;

        /*!!!после сорев сделать
        drive.followTrajectory(myTrajectorySpline31);

        Robot.servo.setPosition(0.8);
        timer.reset();
        while (timer.milliseconds() < 500 && !isStopRequested());*/

        /*drive.followTrajectory(myTrajectorySpline5);

        new_thread.launch4 = true;
        timer.reset();
        while (timer.milliseconds() < 1500 && !isStopRequested()) ;
        Robot.servo.setPosition(1.2);
        new_thread.launch2 = true;
        timer.reset();
        while (timer.milliseconds() < 3500 && !isStopRequested()) ;

        drive.followTrajectory(myTrajectorySpline6);
        Robot.servo.setPosition(0.8);
        timer.reset();
        while (timer.milliseconds() < 750 && !isStopRequested());
        drive.followTrajectory(myTrajectorySpline7);*/


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


