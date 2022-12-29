package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AprilTagAutonomousInitDetectionExample.FEET_PER_METER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Auto_Middle_Right2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotNW Robot = new RobotNW();
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;
        ElapsedTime timer = new ElapsedTime();
        Robot.init(hardwareMap, telemetry, this);
        final double FEET_PER_METER = 3.28084;
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;
        double y = -6;
        double x = -40;
        // UNITS ARE METERS
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
                    telemetry.addLine( "\nBut we HAVE seen the tag before; last seen at:");
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
            telemetry.addLine("FIRST");
            telemetry.update();
            y = 10;
        } else if (tagOfInterest.id == MIDDLE) {
            telemetry.addLine("SECOND");
            telemetry.update();
            y = -17;
        } else {
            telemetry.addLine(  "THIRD");
            telemetry.update();
            y = -37;
        }

        Pose2d StartPose = new Pose2d(-64, -41);
        drive.setPoseEstimate(StartPose);
        Trajectory myTrajectorySpline = drive.trajectoryBuilder(StartPose)
                .lineTo(new Vector2d(-45, -33))
                .build();
        Trajectory myTrajectorySpline2 = drive.trajectoryBuilder(myTrajectorySpline.end())
                .lineTo(new Vector2d(-36.5, -12.75))
                .build();
        Trajectory myTrajectorySpline3 = drive.trajectoryBuilder(myTrajectorySpline2.end())
                .lineTo(new Vector2d(-44, -30))
                .build();
        Trajectory myTrajectorySpline35 = drive.trajectoryBuilder(myTrajectorySpline3.end())
                .lineTo(new Vector2d(-38, -30))
                .build();
        Trajectory myTrajectorySpline4 = drive.trajectoryBuilder(myTrajectorySpline3.end())
                .splineTo(new Vector2d(-26.5,-46), Math.toRadians(-90))
                .build();
        Trajectory myTrajectorySpline5 = drive.trajectoryBuilder(myTrajectorySpline4.end())
                .lineTo(new Vector2d(-23.5, -32))
                .build();
        Trajectory myTrajectorySpline6 = drive.trajectoryBuilder(new Pose2d(-23.5, 32, Math.toRadians(125)))
                .lineToConstantHeading(new Vector2d(-23, -36))
                .build(); // проезд вперед к дж
        Trajectory myTrajectorySpline7 = drive.trajectoryBuilder(myTrajectorySpline6.end())
                .lineTo(new Vector2d(-17, -29))
                .build(); // отъезд назад
        Trajectory myTrajectorySpline8 = drive.trajectoryBuilder(new Pose2d(-17, 29, Math.toRadians(-90)))
                .lineTo(new Vector2d(-20, y))
                .build(); //парковка
        waitForStart();
        Robot.servo.setPosition(0.8); //???

        //Robot.EL.SetPosition_for_Elevator("middle");
        timer.reset();
        while(timer.milliseconds() < 1200 && !isStopRequested());
        Robot.EL.Elevator.setPower(1);
        timer.reset();
        while(timer.milliseconds() < 2000 && !isStopRequested());
        Robot.EL.Elevator.setPower(0.1);

        drive.followTrajectory(myTrajectorySpline);
        drive.followTrajectory(myTrajectorySpline2);

        Robot.servo.setPosition(0.4); //???
        timer.reset();
        while(timer.milliseconds() < 1000 && !isStopRequested());

        drive.followTrajectory(myTrajectorySpline3);
        drive.followTrajectory(myTrajectorySpline35);
         /*drive.followTrajectory(myTrajectorySpline4);

        //Robot.EL.SetPosition_for_Elevator("begin");

        Robot.EL.Elevator.setPower(-1);
        timer.reset();
        while(timer.milliseconds() < 1200 && !isStopRequested());
        Robot.EL.Elevator.setPower(0);
        Robot.servo.setPosition(0.8); //??
        timer.reset();
        while(timer.milliseconds() < 1000 && !isStopRequested());
        Robot.EL.Elevator.setPower(1);
        timer.reset();
        while(timer.milliseconds() < 1800 && !isStopRequested());
        Robot.EL.Elevator.setPower(0.1);

        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(myTrajectorySpline8);
        drive.followTrajectory(myTrajectorySpline5);
//        Robot.EL.SetPosition_for_Elevator("middle");
        drive.turn(Math.toRadians(-45));
        drive.followTrajectory(myTrajectorySpline6);
        Robot.servo.setPosition(0.4); //???
        timer.reset();
        while(timer.milliseconds() < 1000 && !isStopRequested());
        drive.followTrajectory(myTrajectorySpline7);
        //drive.turn(Math.toRadians(45));
        //drive.followTrajectory(myTrajectorySpline8);
*/
    }
    void tagToTelemetry (AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    }