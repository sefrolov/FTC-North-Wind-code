package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous
public class EncAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        List<Double> pos;
        waitForStart();
        while(opModeIsActive()) {
            pos = drive.getWheelPositions();
            telemetry.addData("motor_lf", pos.get(0));
            telemetry.addData("motor_lb", pos.get(1));
            telemetry.addData("motor_rb", pos.get(2));
            telemetry.addData("motor_rf", pos.get(3));
            telemetry.update();
            if (abs(gamepad1.left_stick_y) > 0.03) {
                drive.setMotorPowers(-gamepad1.left_stick_y, -gamepad1.left_stick_y, -gamepad1.left_stick_y, -gamepad1.left_stick_y);
            }
            else{
                drive.setMotorPowers(0, 0, 0, 0);
            }
        }
    }
}
