package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.AprilTagAutonomousInitDetectionExample.FEET_PER_METER;

import static java.lang.Double.max;
import static java.lang.Math.PI;

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
import java.util.List;
import java.util.Vector;

@Autonomous
public class auto_3encodera extends LinearOpMode {

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
        new_thread.HM = hardwareMap;
        new_thread.init();

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



        waitForStart();
        new_thread.start();
        new_thread.setTargetAng(2080);
        new_thread.EL.changeover.setPosition(0.96);
        while (Robot.WB.MotorLF.getCurrentPosition() < 1200){
            Robot.WB.applySpeed(0, max(1., (Robot.WB.MotorLF.getCurrentPosition() - 1200) / 400.), 0, 1.5);
        };
        /*telemetry.addData("lf", Robot.WB.MotorLF.getCurrentPosition());
        telemetry.update();*/
        Robot.WB.applyBreak();
        timer.reset();
        while (timer.milliseconds() < 500 && !isStopRequested());
        Robot.WB.applySpeed(1,0.4,0, 1.5);
        while (Robot.WB.MotorLF.getCurrentPosition() >= 1200 && Robot.WB.MotorLF.getCurrentPosition() < 2100){
            Robot.WB.applySpeed(1,0.3,0, 1.5);
        }
        Robot.WB.applyBreak();
        while (Robot.IMU.getAngle() > 0) {
            Robot.WB.applySpeed(0, 0, 0.2, 1);
            telemetry.addData("imu", Robot.IMU.getAngle());
            telemetry.update();

        }
        Robot.WB.applyBreak();
        Robot.servo.setPosition(0.8);

        timer.reset();
        while (timer.milliseconds() < 1500 && !isStopRequested());

        new_thread.change_per(100);
        new_thread.setTargetAng(800);

        while (Robot.WB.MotorLF.getCurrentPosition() > 2225){
            Robot.WB.applySpeed(0, -1, 0, 3);
        };
        Robot.WB.applyBreak();
        timer.reset();
        while (timer.milliseconds() < 500 && !isStopRequested()) {
            Robot.WB.applyBreak();
        }
        while (Robot.IMU.getAngle() > -0.46 * PI) {
            Robot.WB.applySpeed(0, 0, 0.52 * PI + Robot.IMU.getAngle(), 2);
            telemetry.addData("imu", Robot.IMU.getAngle());
            telemetry.update();
        }
        Robot.WB.applyBreak();
        timer.reset();
        while (timer.milliseconds() < 100 && !isStopRequested());
        /*List<Double> pos = drive.getWheelPositions();
        telemetry.addData("motor_lf", pos.get(0));
        //telemetry.update();*/
        double n = Robot.WB.MotorLF.getCurrentPosition();
        telemetry.addData("lf", Robot.WB.MotorLF.getCurrentPosition());
        telemetry.addData("n", n);
        telemetry.update();

      //  double n = Robot.WB.MotorLF.getCurrentPosition();
      Robot.WB.applySpeed(0, -1, 0, 2.5);
        while ((n - Robot.WB.MotorLF.getCurrentPosition()) < 1300){
            Robot.WB.applySpeed(0, -1, 0, 2.5);
        }
        Robot.WB.applyBreak();
        new_thread.setTargetAng(270);
        timer.reset();
        while (timer.milliseconds() < 200 && !isStopRequested());
        Robot.servo.setPosition(1);
        timer.reset();
        while (timer.milliseconds() < 200 && !isStopRequested());
        new_thread.setTargetAng(2080);
        new_thread.setTagetChangeAng_timer(0.5, 0);
        n = Robot.WB.MotorLF.getCurrentPosition();
        while ((n - Robot.WB.MotorLF.getCurrentPosition()) > -800){
            Robot.WB.applySpeed(-0.4, max(1, (1000 + n - Robot.WB.MotorLF.getCurrentPosition()) / 300), 0, 1.5);
        }



        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            //y = 92;
            telemetry.addLine("FIRST");
            telemetry.update();
        } else if (tagOfInterest.id == MIDDLE) {
            //y = 61;
            telemetry.addLine("SECOND");
            telemetry.update();
        } else {
            //y = 39;
            telemetry.addLine("THIRD");
            telemetry.update();

        }
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


