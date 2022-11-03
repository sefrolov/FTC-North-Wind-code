package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrainMecanum {
    DcMotor MotorRF, MotorLF, MotorRB, MotorLB;
    double x = 0, y = 0;
    double[] motorEnc = {0, 0, 0, 0};

    public void init(HardwareMap HM){
        MotorLB = HM.get(DcMotor.class, "motor_lb");
        MotorRB = HM.get(DcMotor.class, "motor_rb");
        MotorLF = HM.get(DcMotor.class, "motor_lf");
        MotorRF = HM.get(DcMotor.class, "motor_rf");

        MotorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorLF.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void applySpeed(double x, double y, double rotate, double koef){
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

        if(dev > 1){
            lf /= dev;
            lb /= dev;
            rf /= dev;
            rb /= dev;
        }

        MotorLB.setPower(lb / koef);
        MotorRB.setPower(rb / koef);
        MotorLF.setPower(lf / koef);
        MotorRF.setPower(rf / koef);
    }

    public void applyBreak(){
        MotorLB.setPower(0);
        MotorRB.setPower(0);
        MotorLF.setPower(0);
        MotorRF.setPower(0);;
    }

    public void updatePos(){

    }
}
