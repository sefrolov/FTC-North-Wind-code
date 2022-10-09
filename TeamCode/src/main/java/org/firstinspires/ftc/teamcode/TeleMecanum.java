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

@TeleOp(name = "main_tele_rev")
public class sample extends LinearOpMode {
    DcMotor MotorRF, MotorLF, MotorRB, MotorLB, liftMotor;
    Servo servo;
    boolean open = false;
    double rotation_speed = 0, mod = 0;
    double speedLF = 0, speedRF = 0, speedLB = 0, speedRB = 0;
    ElapsedTime button_timer = new ElapsedTime();
    //vec2 JoysticDir;

    public void runOpMode(){
        MotorLB = hardwareMap.get(DcMotor.class, "motor_lb");
        MotorRB = hardwareMap.get(DcMotor.class, "motor_rb");
        MotorLF = hardwareMap.get(DcMotor.class, "motor_lf");
        MotorRF = hardwareMap.get(DcMotor.class, "motor_rf");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        servo = hardwareMap.get(Servo.class, "cap");

        MotorRB.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        button_timer.reset();

        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotate = gamepad1.right_trigger - gamepad1.left_trigger;
            double lift = gamepad2.left_stick_y;
            double lf = y + x + rotate;
            double lb = y - x + rotate;
            double rf = y - x - rotate;
            double rb = y + x - rotate;
            double[] powers = {lf, lb, rf, rb};
            double max = 0;
            for (int i = 0; i < 3; i++) {
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
            liftMotor.setPower(lift);
            telemetry.addData("lb", lb);
            telemetry.addData("lf", lf);
            telemetry.addData("rf", rf);
            telemetry.addData("rb", rb);
            telemetry.update();
        }
    }
}