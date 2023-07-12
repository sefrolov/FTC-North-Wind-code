package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.AprilTagAutonomousInitDetectionExample.FEET_PER_METER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class auto_5high_side extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotNW Robot = new RobotNW();
        ElapsedTime timer = new ElapsedTime();
        elevatorNewThreadMiddle_Red new_thread = new elevatorNewThreadMiddle_Red();
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

        Pose2d StartPose = new Pose2d(-66, 41, Math.toRadians(0));
        new_thread.HM = hardwareMap;
        new_thread.init();

        //Trajectory myTrajectorySpline = drive.trajectoryBuilder(StartPose)
            //    .lineToSplineHeading(new Pose2d(/*-16*/-24, 28, Math.toRadians(0)))
              //  .splineToSplineHeading(new Pose2d(-/*10*/7.75, 25/*21.75*/, Math.toRadians(-45)), Math.toRadians(-45))
                //.splineToSplineHeading(new Pose2d(-8.75, 30., Math.toRadians(-65)), Math.toRadians(-65))
               // .splineToSplineHeading(new Pose2d(-8.5, 30., Math.toRadians(-65)), Math.toRadians(-65))
          //      .build();
        Trajectory myTrajectorySpline = drive.trajectoryBuilder(StartPose)
                .lineToSplineHeading(new Pose2d(-12, 30, Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(95, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        Trajectory myTrajectorySpline12 = drive.trajectoryBuilder(myTrajectorySpline.end())
                .lineToSplineHeading(new Pose2d(-7.5, 21, Math.toRadians(-90)))
                .build();

        Trajectory myTrajectorySpline2 = drive.trajectoryBuilder(myTrajectorySpline12.end()/*myTrajectorySpline.end()*/)
                //.splineToSplineHeading(new Pose2d(-13, 50, Math.toRadians(-90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-13.5, 64.5, Math.toRadians(-90)))
                .build(); //стопка
        Trajectory myTrajectorySpline3 = drive.trajectoryBuilder(myTrajectorySpline2.end())
                //.splineToSplineHeading(new Pose2d(-10, 48, Math.toRadians(-45)), Math.toRadians(-45))
                .lineToSplineHeading(new Pose2d(-7., 20, Math.toRadians(-90)))
                .build(); //2 дж
        Trajectory myTrajectorySpline4 = drive.trajectoryBuilder(myTrajectorySpline3.end())
                //.splineToSplineHeading(new Pose2d(-13, 50, Math.toRadians(-90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(/*-19.3*/ -13.5, 64.5, Math.toRadians(-90)))
                .build(); //стопка
        Trajectory myTrajectorySpline5 = drive.trajectoryBuilder(myTrajectorySpline4.end())
                //.splineToSplineHeading(new Pose2d(-10, 48, Math.toRadians(-45)), Math.toRadians(-45))
                .lineToSplineHeading(new Pose2d(-6., 20, Math.toRadians(-90)))
                .build(); //3 дж
        Trajectory myTrajectorySpline6 = drive.trajectoryBuilder(myTrajectorySpline5.end())
                //.splineToSplineHeading(new Pose2d(-13, 50, Math.toRadians(-90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(/*-19.3*/ -13.5, 64.5, Math.toRadians(-90)))
                .build(); //стопка
        Trajectory myTrajectorySpline7 = drive.trajectoryBuilder(myTrajectorySpline6.end())
                //.splineToSplineHeading(new Pose2d(-10, 48, Math.toRadians(-45)), Math.toRadians(-45))
                .lineToSplineHeading(new Pose2d(-6., 20, Math.toRadians(-90)))
                .build(); //4 дж
        Trajectory myTrajectorySpline8 = drive.trajectoryBuilder(myTrajectorySpline7.end())
                //.splineToSplineHeading(new Pose2d(-13, 50, Math.toRadians(-90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(/*-19.3*/ -13.5, 64.5, Math.toRadians(-90)))
                .build(); //стопка
        Trajectory myTrajectorySpline9 = drive.trajectoryBuilder(myTrajectorySpline8.end())
                //.splineToSplineHeading(new Pose2d(-10, 48, Math.toRadians(-45)), Math.toRadians(-45))
                .lineToSplineHeading(new Pose2d(-6., 20, Math.toRadians(-90)))
                .build(); //5 дж
        Trajectory myTrajectorySpline10 = drive.trajectoryBuilder(myTrajectorySpline7.end())
                //.splineToSplineHeading(new Pose2d(-13, 50, Math.toRadians(-90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(/*-19.3*/ -13.5, 64.5, Math.toRadians(-90)))
                .build(); //стопка
        Trajectory myTrajectorySpline11 = drive.trajectoryBuilder(myTrajectorySpline8.end())
                //.splineToSplineHeading(new Pose2d(-10, 48, Math.toRadians(-45)), Math.toRadians(-45))
                .lineToSplineHeading(new Pose2d(-7., 20, Math.toRadians(-90)))
                .build(); //6 дж


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
            y = 60;
            telemetry.addLine("FIRST");
            telemetry.update();
        } else if (tagOfInterest.id == MIDDLE) {
            y = 37;
            telemetry.addLine("SECOND");
            telemetry.update();
        } else {
            y = 16;
            telemetry.addLine("THIRD");
            telemetry.update();

        }
        Trajectory myTrajectorySpline_park = drive.trajectoryBuilder(myTrajectorySpline11.end())
                .lineToSplineHeading(new Pose2d(-10, y, Math.toRadians(-90)))
                .build();


        waitForStart();
        new_thread.start();
        /*new_thread.setTagetChangeAng_timer(0.53, 1000);*/
        /*new_thread.setTagetChangeAng_timer(0.945, 1000);
        new_thread.setTagetLiftAng_timer(2120, 800);
        new_thread.setTagetSideAng_timer(1430, 800);*/
        new_thread.setTagetChangeAng_timer(0.53, 1300);
        new_thread.setTagetLiftAng_timer(2130, 1100);
        new_thread.setTagetSideAng_timer(1430, 1100);
        drive.followTrajectory(myTrajectorySpline);
        drive.followTrajectory(myTrajectorySpline12);
        new_thread.setTagetSideAng_timer(1415, 0);
        telemetry.update();

        Robot.servo.setPosition(0.8);

        timer.reset();
        while (timer.milliseconds() < 150 && !isStopRequested());

        new_thread.setTagetChangeAng_timer(0.175, 750);
        new_thread.setTagetLiftAng_timer(800, 600);
        new_thread.setTagetSideAng_timer(200, 600);

        drive.followTrajectory(myTrajectorySpline2);
        new_thread.setTargetAng(270);
        timer.reset();
        while (timer.milliseconds() < 100 && !isStopRequested());
        Robot.servo.setPosition(1);

        timer.reset();
        while (timer.milliseconds() < 150 && !isStopRequested());
        new_thread.setTargetAng_side(540);
        new_thread.setTargetAng(970);
        new_thread.setTagetChangeAng_timer(0.53, 1000);
        new_thread.setTagetLiftAng_timer(2120, 800);
        new_thread.setTagetSideAng_timer(1430, 800);
        drive.followTrajectory(myTrajectorySpline3);
        new_thread.setTagetSideAng_timer(1415, 0);
        telemetry.update();

        Robot.servo.setPosition(0.8);

        timer.reset();
        while (timer.milliseconds() < 100 && !isStopRequested());

        new_thread.setTagetChangeAng_timer(0.175, 650);
        new_thread.setTagetLiftAng_timer(750, 600);
        new_thread.setTagetSideAng_timer(175, 600);

        drive.followTrajectory(myTrajectorySpline4);
        new_thread.setTargetAng(220);
        timer.reset();
        while (timer.milliseconds() < 100 && !isStopRequested());
        Robot.servo.setPosition(1);

        timer.reset();
        while (timer.milliseconds() < 150 && !isStopRequested());
        new_thread.setTargetAng_side(540);
        new_thread.setTargetAng(970);
        new_thread.setTagetChangeAng_timer(0.53, 1000);
        new_thread.setTagetLiftAng_timer(2120, 800);
        new_thread.setTagetSideAng_timer(1430, 800);
        drive.followTrajectory(myTrajectorySpline5);
        new_thread.setTagetSideAng_timer(1415, 0);
        telemetry.update();

        Robot.servo.setPosition(0.8);

        timer.reset();
        while (timer.milliseconds() < 100 && !isStopRequested());

        new_thread.setTagetChangeAng_timer(0.175, 650);
        new_thread.setTagetLiftAng_timer(700, 600);
        new_thread.setTagetSideAng_timer(150, 600);

        drive.followTrajectory(myTrajectorySpline6);
        new_thread.setTargetAng(170);
        timer.reset();
        while (timer.milliseconds() < 100 && !isStopRequested());
        Robot.servo.setPosition(1);

        timer.reset();
        while (timer.milliseconds() < 150 && !isStopRequested());
        new_thread.setTargetAng_side(540);
        new_thread.setTargetAng(970);
        new_thread.setTagetChangeAng_timer(0.53, 1000);
        new_thread.setTagetLiftAng_timer(2120, 800);
        new_thread.setTagetSideAng_timer(1430, 800);
        drive.followTrajectory(myTrajectorySpline7);
        new_thread.setTagetSideAng_timer(1415, 0);
        telemetry.update();

        Robot.servo.setPosition(0.8);

        timer.reset();
        while (timer.milliseconds() < 100 && !isStopRequested());

        new_thread.setTagetChangeAng_timer(0.175, 650);
        new_thread.setTagetLiftAng_timer(650, 600);
        new_thread.setTagetSideAng_timer(125, 600);

        drive.followTrajectory(myTrajectorySpline8);
        new_thread.setTargetAng(120);
        timer.reset();
        while (timer.milliseconds() < 100 && !isStopRequested());
        Robot.servo.setPosition(1);

        timer.reset();
        while (timer.milliseconds() < 150 && !isStopRequested());
        new_thread.setTargetAng_side(540);
        new_thread.setTargetAng(970);
        new_thread.setTagetChangeAng_timer(0.53, 1000);
        new_thread.setTagetLiftAng_timer(2120, 800);
        new_thread.setTagetSideAng_timer(1430, 800);
        drive.followTrajectory(myTrajectorySpline9);
        new_thread.setTagetSideAng_timer(1415, 0);
        telemetry.update();
        Robot.servo.setPosition(0.8);

        timer.reset();
        while (timer.milliseconds() < 100 && !isStopRequested());

        new_thread.setTagetChangeAng_timer(0.175, 650);
        new_thread.setTagetLiftAng_timer(650, 600);
        new_thread.setTagetSideAng_timer(125, 600);

        drive.followTrajectory(myTrajectorySpline10);
        new_thread.setTargetAng(70);
        timer.reset();
        while (timer.milliseconds() < 100 && !isStopRequested());
        Robot.servo.setPosition(1);

        timer.reset();
        while (timer.milliseconds() < 150 && !isStopRequested());
        new_thread.setTargetAng_side(540);
        new_thread.setTargetAng(970);
        new_thread.setTagetChangeAng_timer(0.53, 1000);
        new_thread.setTagetLiftAng_timer(2120, 800);
        new_thread.setTagetSideAng_timer(1430, 800);
        drive.followTrajectory(myTrajectorySpline11);
        new_thread.setTagetSideAng_timer(1415, 0);
        telemetry.update();

        Robot.servo.setPosition(0.8);

        timer.reset();
        while (timer.milliseconds() < 100 && !isStopRequested());

        new_thread.setTagetChangeAng_timer(0.175, 650);

        new_thread.setTagetSideAng_timer(0, 250);
        new_thread.setTagetLiftAng_timer(0, 250);
        drive.followTrajectory(myTrajectorySpline_park);


        timer.reset();
        while (timer.milliseconds() < 50 && !isStopRequested());
        new_thread.EL.Elevator.setPower(0);
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
