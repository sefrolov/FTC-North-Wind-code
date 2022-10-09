package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TeleMecanum extends LinearOpMode {
    DcMotor MotorRF, MotorLF, MotorRB, MotorLB;

    public void runOpMode(){
        MotorLB = hardwareMap.get(DcMotor.class, "motor_lb");
        MotorRB = hardwareMap.get(DcMotor.class, "motor_rb");
        MotorLF = hardwareMap.get(DcMotor.class, "motor_lf");
        MotorRF = hardwareMap.get(DcMotor.class, "motor_rf");

        MotorRB.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRF.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("","Init complited");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotate = gamepad1.right_trigger - gamepad1.left_trigger;
            double lf = y + x + rotate;
            double lb = y - x + rotate;
            double rf = y - x - rotate;
            double rb = y + x - rotate;
            double[] powers = {lf, lb, rf, rb};
            double max = 0;
            for (int i = 0; i < 4; i++) {
                if (Math.abs(powers[i]) > max)
                    max = Math.abs(powers[i]);
            }
            if (max > 1) {
                lf /= max;
                lb /= max;
                rf /= max;
                rb /= max;
            }
            MotorLB.setPower(lb);
            MotorRB.setPower(rb);
            MotorLF.setPower(lf);
            MotorRF.setPower(rf);
            telemetry.addData("lb", lb);
            telemetry.addData("lf", lf);
            telemetry.addData("rf", rf);
            telemetry.addData("rb", rb);
            telemetry.update();
        }
    }
}