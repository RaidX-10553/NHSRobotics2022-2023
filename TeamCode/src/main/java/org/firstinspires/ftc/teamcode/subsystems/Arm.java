package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {

    DcMotorEx armMotor;

    public Arm(DcMotorEx armMotor) {
        this.armMotor = armMotor;

    }
    public void Raise() {
        this.armMotor.setTargetPosition(1120);
        this.armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        this.armMotor.setVelocity(600);
    }
    public void Lower() {
        this.armMotor.setTargetPosition(-armMotor.getCurrentPosition());
        this.armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        this.armMotor.setVelocity(600);
        this.armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }
}
