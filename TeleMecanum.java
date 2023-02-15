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
    DcMotor MotorRF, MotorLF, MotorRB, MotorLB, LED, lift;
    Servo servo;
    elevatorThreadMiddle new_thread = new elevatorThreadMiddle();
    public void runOpMode(){
        //servo = hardwareMap.get(Servo.class, "servo");
        //LED = hardwareMap.get(DcMotor.class, "led");
        //lift = hardwareMap.get(DcMotor.class, "elevator");
        MotorLB = hardwareMap.get(DcMotor.class, "motor_lb");
        MotorRB = hardwareMap.get(DcMotor.class, "motor_rb");
        MotorLF = hardwareMap.get(DcMotor.class, "motor_lf");
        MotorRF = hardwareMap.get(DcMotor.class, "motor_rf");

        MotorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorLF.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("","Init complited");
        telemetry.update();

        waitForStart();

        //LED.setPower(0.5);
        while(opModeIsActive()){
            double koef = 1.;
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double y_el = -gamepad2.right_stick_y;
            double rotate = gamepad1.right_trigger - gamepad1.left_trigger;
            double lf = y + x + rotate;
            double lb = y - x + rotate;
            double rf = y - x - rotate;
            double rb = y + x - rotate;
            double[] powers = {lf, lb, rf, rb};
            double dev = 0.;
            for (int i = 0; i < 4; i++) {
                if (Math.abs(powers[i]) > dev)
                    dev = Math.abs(powers[i]);
            }

            //dev /= 0.95 * sqrt(x * x + y * y);

            if(dev > 1){
                lf /= dev;
                lb /= dev;
                rf /= dev;
                rb /= dev;
            }

            //lf += rotate;
            //lb += rotate;
            //rf -= rotate;
            //rb -= rotate;


            if(gamepad1.a) koef = 3.;
            else if(gamepad1.b) koef = 2.;
            else koef = 1.;

            MotorLB.setPower(lb / koef);
            MotorRB.setPower(rb / koef);
            MotorLF.setPower(lf / koef);
            MotorRF.setPower(rf / koef);
            //lift.setPower(y_el);

            /*if(gamepad2.x){
                servo.setPosition(-0.8);
            }
            else if(gamepad2.a){
                servo.setPosition(0.4);
            }
            else if(gamepad2.b){
                servo.setPosition(1.2);
            }
            lift.setPower(-gamepad1.right_stick_y);
*/
            telemetry.addData("lb", lb);
            telemetry.addData("lf", lf);
            telemetry.addData("rf", rf);
            telemetry.addData("rb", rb);
            telemetry.update();
        }
    }
}