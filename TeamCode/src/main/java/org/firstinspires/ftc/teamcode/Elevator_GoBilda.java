package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Elevator_GoBilda {
    double TICS_PER_REV = 384.5;
    DcMotor Elevator;

    public void initElevator(HardwareMap HM) {
        Elevator = HM.get(DcMotor.class, "elevator");
        Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Apply_Speed_for_Elevator(double y_elevator) {
        double elevator_speed = y_elevator;
        Elevator.setPower(elevator_speed);
    }

    public void SetPosition_for_Elevator(String position) {
        if (position == "begin") {
            while (Elevator.getCurrentPosition() >= 0) {
                Elevator.setPower(-1);
            }
        } else if (position == "low") {
            while (Elevator.getCurrentPosition() <= 100) {
                Elevator.setPower(1);
            }
        } else if (position == "middle") {
            while (Elevator.getCurrentPosition() <= 300) {
                Elevator.setPower(1);
            }
        } else if (position == "high") {
            while (Elevator.getCurrentPosition() <= 500) {
                Elevator.setPower(1);
            }
        }
    }
}



