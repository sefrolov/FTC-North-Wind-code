package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MainTele")
public class AutoTest extends LinearOpMode {
    RobotNW Robot = new RobotNW();

    public void runOpMode() {
        Robot.init(hardwareMap, telemetry, this);
        telemetry.addData("", "Init complited");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double koef = 1.;
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotate = gamepad1.right_trigger - gamepad1.left_trigger;

            if (gamepad1.a) koef = 2.5;
            else if (gamepad1.b) koef = 5. / 3.;
            else koef = 1.;

            Robot.WB.applySpeed(x, y, rotate, koef);

            if (gamepad2.b) {
                Robot.servo.setPosition(0.16);
            } else if (gamepad2.a) {
                Robot.servo.setPosition(0.4);
            } else if (gamepad2.x) {
                Robot.servo.setPosition(0.8);
            } else if (gamepad2.right_trigger > 0.1) {
                Robot.servo.setPosition(0.8 - gamepad2.right_trigger * 0.8);
            }

            telemetry.addData("Angle", Robot.IMU.getPositiveAngle());
            telemetry.addData("lb", Robot.WB.MotorLB.getCurrentPosition());
            telemetry.addData("lf", Robot.WB.MotorLF.getCurrentPosition());
            telemetry.addData("rf", Robot.WB.MotorRF. getCurrentPosition());
            telemetry.addData("rb", Robot.WB.MotorRB.getCurrentPosition());
            telemetry.update();
        }

        //telemetry.addData("x", Robot.WB.getPos());
        //telemetry.addData("y", Robot.WB.getPos());
        //telemetry.update();
    }
}
