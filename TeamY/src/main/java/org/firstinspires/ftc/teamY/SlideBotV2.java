package org.firstinspires.ftc.teamY;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamY.drive.SampleMecanumDrive;

@TeleOp(name = "Teleop SlideV2", group = "TeleOp")
public class SlideBotV2 extends LinearOpMode {
    //Claw
    Servo claw1;

    DcMotorEx Arm;

    //Drive
    SampleMecanumDrive mecanumDrive;



    private double driveValue = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        //Drive
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        //Claw
        claw1 = hardwareMap.get(Servo.class, "claw1");


        Arm = hardwareMap.get(DcMotorEx.class, "arm");


        waitForStart();


        //Code is looped inside this while loop
        while (opModeIsActive()) {

           //Slow Driving
            if (gamepad1.x) {
                telemetry.addData("Mode", "PRECISE");
                telemetry.update();
                driveValue = 0.4;
            }

            //Reversed Driving
            if (gamepad1.b) {
                telemetry.addData("Mode", "REVERSED");
                telemetry.update();
                driveValue = -1;
            }

            //Normal Driving
            if (gamepad1.a) {
                telemetry.addData("Mode", "Normal");
                telemetry.update();
                driveValue = 1;
            }


            //Drive Code Using RR
            mecanumDrive.setDrivePower(
                    new Pose2d(gamepad1.left_stick_y * driveValue,
                            gamepad1.right_stick_x *driveValue,
                            gamepad1.left_stick_x *driveValue));
            mecanumDrive.updatePoseEstimate();



            //Arm Control
            double x = -gamepad2.left_stick_y;
            Arm.setPower(x);


            //Range is between [0.0 and 1.0] 0.5 being the center
            //This works
            if (gamepad2.left_bumper) {
                claw1.setPosition(0.5);
            }

            if (gamepad2.right_bumper) {
               claw1.setPosition(0);

            }



        }

    }

}