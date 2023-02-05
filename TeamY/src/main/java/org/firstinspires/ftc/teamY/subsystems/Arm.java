package org.firstinspires.ftc.teamY.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {

    DcMotorEx armMotor;
    int placeHolder;


    public Arm(DcMotorEx armMotor) {
        this.armMotor = armMotor;
        this.armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void High() {
        this.armMotor.setTargetPosition(9200);
        this.armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        this.armMotor.setVelocity(1800);
    }
    public void Five() {
        this.armMotor.setTargetPosition(1270);
        this.armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        this.armMotor.setVelocity(1200);
    }
    public void Four() {
        this.armMotor.setTargetPosition(1042);
        this.armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        this.armMotor.setVelocity(1200);
    }
    public void Three() {
        this.armMotor.setTargetPosition(601);
        this.armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        this.armMotor.setVelocity(1200);
    }
    public void SlightLower(){

        this.armMotor.setTargetPosition((armMotor.getCurrentPosition())-5000);
        this.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.armMotor.setVelocity(1800);
    }
    public void Lower() {
        this.armMotor.setTargetPosition(-armMotor.getCurrentPosition());
        this.armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        this.armMotor.setVelocity(1800);
        //og was 600
    }
}
