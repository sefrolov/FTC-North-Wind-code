package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.max;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Elevator_GoBilda {
    double TICS_PER_REV = 384.5;
    DcMotor Elevator;
    Servo changeover;
    int CurAng, TargetAng, LastErAng, ErAng;
    double p_coef = 0.1, d_coef = 0, p, d;

    public void initElevator(HardwareMap HM) {
        Elevator = HM.get(DcMotor.class, "elevator");
        changeover = HM.get(Servo.class, "changeover");
        Elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public int getPos(){return Elevator.getCurrentPosition();}

    public void addTargetAng(int Ang) {TargetAng += Ang;}

    public int getTargetAng() { return TargetAng; }

    public void setTargetAng(int Ang) { TargetAng = Ang; }

    public void Lift_PID_controller(){
        CurAng = getPos();
        ErAng = TargetAng - CurAng;
        p = ErAng * p_coef;
        d = (ErAng - LastErAng) * d_coef;

        Elevator.setPower(max(p + d, -0.2));
        LastErAng = ErAng;
    }
}

class elevatorThreadMiddle extends Thread {
    HardwareMap HM;
    Elevator_GoBilda EL = new Elevator_GoBilda();
    ElapsedTime timer = new ElapsedTime(), change_timer = new ElapsedTime();
    boolean launch = false, launch1 = false, launch2 = false, launch3 = false, launch4 = false, launch5 = false, change_flag = false;
    int change_time = 0;

    public void run() {
        EL.initElevator(HM);
        while (!isInterrupted()) {
            EL.Lift_PID_controller();
            if (launch) {
                //telemetry.addData("Поток", "ура, победа");
                //telemetry.update();

                EL.Elevator.setPower(1);
                timer.reset();
                while (timer.milliseconds() < 2300 && !isInterrupted()) ;

                //telemetry.addData("Поток", "УРА, ПОБЕДА");
                //telemetry.update();
                EL.Elevator.setPower(0.);
                /*timer.reset();
                while (timer.milliseconds() < 1200 && !isInterrupted()) ;*/
                launch = false;
            }

            if (launch1) {
                //telemetry.addData("Поток", "ура, победа");
                //telemetry.update();
                timer.reset();
                while (timer.milliseconds() < 1000 && !isInterrupted()) ;
                EL.Elevator.setPower(-0.5);
                timer.reset();
                while (timer.milliseconds() < 100 && !isInterrupted()) ;
                EL.Elevator.setPower(0);
                launch1 = false;
            }

            if (launch2) {
                //telemetry.addData("Поток", "ура, победа");
                //telemetry.update();
                EL.Elevator.setPower(1);
                timer.reset();
                while (timer.milliseconds() < 7000 && !isInterrupted()) ;
            }

            if (launch3) {
                EL.Elevator.setPower(1);
                timer.reset();
                while (timer.milliseconds() < 1600 && !isInterrupted()) ;

                EL.Elevator.setPower(0.1);
                launch3 = false;
            }

            if (launch4) {
                EL.Elevator.setPower(-1);
                timer.reset();
                while (timer.milliseconds() < 1200 && !isInterrupted()) ;
                EL.Elevator.setPower(0);
                launch4 = false;
            }

            if (launch5) {
                EL.Elevator.setPower(1);
                timer.reset();
                while (timer.milliseconds() < 1600 && !isInterrupted());
                EL.Elevator.setPower(0.1);
                launch5 = false;
            }

            if (change_flag && change_timer.milliseconds() > change_time){
                if (EL.changeover.getPosition() > 0.5) EL.changeover.setPosition(0.05);
                else EL.changeover.setPosition(0.88);
                change_flag = false;
            }
        }
    }

    void change_per(int time){
        change_timer.reset();
        change_time = time;
        change_flag = true;
    }
}

class elevatorNewThreadMiddle extends Thread {
    HardwareMap HM;
    Elevator_GoBilda EL = new Elevator_GoBilda();
    ElapsedTime timer = new ElapsedTime(), change_timer = new ElapsedTime(), lift_timer = new ElapsedTime();
    boolean launch = false, launch1 = false, launch2 = false, launch3 = false, launch4 = false, launch5 = false, launch_end = false, lift_flag = false, change_flag = false;
    int change_time = 0, lift_time = 0, lift_pos = 0;

    public void run() {
        //EL.initElevator(HM);
        while (!isInterrupted()) {
            //EL.Lift_PID_controller();
            /*if (launch) {
                EL.Elevator.setPower(1);
                //EL.changeover.setPosition(0.33); //???
                timer.reset();
                while (timer.milliseconds() < 2500 && !isInterrupted()) ;

                EL.Elevator.setPower(0.1);
                timer.reset();
                while (timer.milliseconds() < 1200 && !isInterrupted()) ;
                launch = false;
            }*/

            //if (launch1) {
               /* timer.reset();
                while (timer.milliseconds() < 1700 && !isInterrupted()) ;*/
                /*EL.changeover.setPosition(0.96);
                timer.reset();
                while (timer.milliseconds() < 1500 && !isInterrupted()) ;
                EL.Elevator.setPower(-1);
                timer.reset();
                while (timer.milliseconds() < 1500 && !isInterrupted()) ;
                EL.Elevator.setPower(0);
                launch1 = false;
            }*/

            /*if (launch2) {
                timer.reset();
                while (timer.milliseconds() < 500 && !isInterrupted()) ;
                EL.Elevator.setPower(1);
                timer.reset();
                while (timer.milliseconds() < 2000 && !isInterrupted()) ;
                EL.Elevator.setPower(0.1);
                launch2 = false;
            }*/

            /*if (launch3) {
                EL.Elevator.setPower(1);
                timer.reset();
                while (timer.milliseconds() < 1800 && !isInterrupted());
                EL.Elevator.setPower(0.1);
                launch3 = false;
            }

            if (launch4) {
                EL.Elevator.setPower(-1);
                EL.changeover.setPosition(0); //???
                timer.reset();
                while (timer.milliseconds() < 1200 && !isInterrupted());
                EL.Elevator.setPower(0);
                launch4 = false;
            }

            if (launch5) {
                EL.Elevator.setPower(1);
                EL.changeover.setPosition(0.66); //???
                timer.reset();
                while (timer.milliseconds() < 1800 && !isInterrupted());
                EL.Elevator.setPower(0.1);
                launch5 = false;
            }

            if (launch_end) {
                EL.Elevator.setPower(-1);
                timer.reset();
                while (timer.milliseconds() < 1000 && !isInterrupted());
                EL.Elevator.setPower(0.1);
                launch5 = false;
            }*/


            if (change_flag && change_timer.milliseconds() > change_time){
                if (EL.changeover.getPosition() > 0.5) EL.changeover.setPosition(0.16);
                else EL.changeover.setPosition(0.96);
                change_flag = false;
            }

            if (lift_flag && lift_timer.milliseconds() > lift_time){
                EL.Elevator.setTargetPosition(lift_pos);
                lift_flag = false;
            }
        }
    }

    public void init(){
        EL.initElevator(HM);
    }

    void change_per(int time){
        change_timer.reset();
        change_time = time;
        change_flag = true;
    }

    void setTagetLiftAng_timer(int pos, int time){
        lift_timer.reset();
        lift_time = time;
        lift_flag = true;
        lift_pos = pos;
    }

    public void addTargetAng(int Ang) {EL.addTargetAng(Ang);}

    public void setTargetAng(int Ang) {EL.setTargetAng(Ang);}

    public int getTargetAng() {return EL.getTargetAng();}

    public int getPos(){return EL.getPos();}
}



