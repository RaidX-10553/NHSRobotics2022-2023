package org.firstinspires.ftc.teamY;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamY.drive.SampleMecanumDrive;

@TeleOp(name = "Teleop SlideV2", group = "TeleOp")
public class SlideBotV2 extends LinearOpMode {
    //Claw
    Servo claw1;
    TouchSensor sensor;
    DcMotorEx Arm;

    //Drive
    SampleMecanumDrive mecanumDrive;

    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    private double driveValue = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        //Drive
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        //Claw
        claw1 = hardwareMap.get(Servo.class, "claw1");

        sensor = hardwareMap.get(TouchSensor.class, "Toucher");


        Arm = hardwareMap.get(DcMotorEx.class, "arm");

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setTargetPosition(0);

        waitForStart();


        //Code is looped inside this while loop
        while (opModeIsActive()) {

            //Try-Catch to prevent possible exception
            try {
             previousGamepad2.copy(currentGamepad2);
             currentGamepad2.copy(gamepad2);

             throw new RobotCoreException("you are a failure");
            }
            catch (RobotCoreException e) {
             // Swallow the possible exception
            }




           //Slow Driving
            if (gamepad1.x) {
                telemetry.addData("Mode", "PRECISE");
                telemetry.update();
                driveValue = 0.5;
            }

            //Reversed Driving
            if (gamepad1.b) {
                telemetry.addData("Mode", "REVERSED");
                telemetry.update();
                driveValue = -0.8;
            }

            //Normal Driving
            if (gamepad1.a) {
                telemetry.addData("Mode", "Normal");
                telemetry.update();
                driveValue = 0.8;
            }


            //Drive Code Using RR
            mecanumDrive.setDrivePower(
                    new Pose2d(-gamepad1.left_stick_y * driveValue,
                            -gamepad1.right_stick_x * driveValue,
                            -gamepad1.left_stick_x * driveValue));
            mecanumDrive.updatePoseEstimate();



            //Arm Control
            double x = gamepad2.left_stick_y;
            Arm.setPower(x);


            //Range is between [0.0 and 1.0] 0.5 being the center
            //This works
            if (gamepad2.left_bumper) {
                claw1.setPosition(0.5);
            }

            if (gamepad2.right_bumper) {
               claw1.setPosition(0);

            }

            if (sensor.isPressed()) {
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("B","k");
                telemetry.update();
            }


            
            //Experimental Claw Toggle
            //Falling Edge Detector
            if (!currentGamepad2.a && previousGamepad2.a) {
                if (claw1.getPosition() == 0) {
                    claw1.setPosition(0.5);
                } else {
                   claw1.setPosition(0);
                }
            }






            






            telemetry.addData("Pos", claw1.getPosition());
            telemetry.addData("Arm extension/test sensor", Arm.getCurrentPosition());
            telemetry.update();



        }

    }

}
