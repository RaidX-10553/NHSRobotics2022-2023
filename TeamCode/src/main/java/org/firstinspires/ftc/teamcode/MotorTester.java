package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Tester", group = "TeleOp")
public class MotorTester extends LinearOpMode {

    //Drive
    DcMotorEx BL;
    DcMotorEx BR;
    DcMotorEx FL;
    DcMotorEx FR;




    @Override
    public void runOpMode() throws InterruptedException {
        //Drive
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FR = hardwareMap.get(DcMotorEx.class, "FR");



        waitForStart();

        //Code is looped inside this while loop
        while (opModeIsActive()) {

            if(gamepad1.dpad_up) {
                FL.setPower(1);
            }

            if(gamepad1.dpad_left) {
                BL.setPower(1);
            }

            if(gamepad1.dpad_down) {
                FR.setPower(1);
            }

            if(gamepad1.dpad_right) {
                BR.setPower(1);
            }






            telemetry.addLine("Values should be positive");
            telemetry.addData("Front Left | Dpad Up", FL.getCurrentPosition());
            telemetry.addData("Back Left | Dpad Left", BL.getCurrentPosition());
            telemetry.addData("Front Right | Dpad Down", FR.getCurrentPosition());
            telemetry.addData("Back Right | Dpad Right", BR.getCurrentPosition());
            telemetry.update();



        }

    }

}
