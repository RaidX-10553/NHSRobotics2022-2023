
package org.firstinspires.ftc.teamY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamY.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamY.subsystems.Arm;
import org.firstinspires.ftc.teamY.subsystems.MarkerDetection;
import org.firstinspires.ftc.teamY.subsystems.ParkingZone;
import org.firstinspires.ftc.teamY.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="BlueLeftAuto", group="Autonomous")
public class BlueLeftAuto extends LinearOpMode {

    /* Declare OpMode members. */

    OpenCvWebcam webcam;

    TouchSensor toucher;

    MarkerDetection pipeline;

    boolean started = false;

    DcMotorEx armMotor;

    Arm arm;

    Servo claw;




    @Override
    public void runOpMode() {
        pipeline = new MarkerDetection();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //Still working on the trajectories, not final
        //Road Runner Trajectory


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        arm = new Arm(armMotor);

        toucher = hardwareMap.get(TouchSensor.class, "Toucher");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        boolean touched = false;

        claw = hardwareMap.get(Servo .class, "claw1");
        //idk
        this.armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        Pose2d startPose = new Pose2d(-35.25, 64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        //flipping
        TrajectorySequence location1 = drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(1);
                })


                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    arm.High();
                })


                .strafeRight(23.5)
                .forward(50)
                //less
                //change 1 to appropriate distance based on tuning
                .turn(Math.toRadians(42))//-42
                .forward(9.6)//og8.7

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    arm.SlightLower();
                })


                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(0);
                    arm.Five();
                })


                .waitSeconds(2)
                .back(8)
                .turn(Math.toRadians(48))//-48
                .strafeRight(1)
                .forward(47.5)


                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(1);
                    arm.High();
                })


                .waitSeconds(1)
                .back(10)
                .turn(Math.toRadians(-90))//90
                .strafeRight(29)//left
                .forward(3)//og 3

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(1);
                    arm.SlightLower();
                })


                //CHANGE TO LEFT OR RIGHT BASED ON DETECTION OR DONT STRAFE AT ALL
                .waitSeconds(2)
                .back(5)
                .strafeLeft(36)
                .build();



        TrajectorySequence location2 = drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(1);
                })


                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    arm.High();
                })


                .strafeRight(23.5)
                .forward(50)
                //less
                //change 1 to appropriate distance based on tuning
                .turn(Math.toRadians(42))//-42
                .forward(9.6)//og8.7

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    arm.SlightLower();
                })


                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(0);
                    arm.Five();
                })


                .waitSeconds(2)
                .back(8)
                .turn(Math.toRadians(48))//-48
                .strafeRight(1)
                .forward(47.5)


                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(1);
                    arm.High();
                })


                .waitSeconds(1)
                .back(10)
                .turn(Math.toRadians(-90))//90
                .strafeRight(29)//left
                .forward(3)//og 3

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(1);
                    arm.SlightLower();
                })


                //CHANGE TO LEFT OR RIGHT BASED ON DETECTION OR DONT STRAFE AT ALL
                .waitSeconds(2)
                .back(5)
                .strafeLeft(12)
                .build();



        TrajectorySequence location3 = drive.trajectorySequenceBuilder(startPose)

                 .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(1);
                 })


                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    arm.High();
                })


                .strafeRight(23.5)
                .forward(50)
                //less
                //change 1 to appropriate distance based on tuning
                .turn(Math.toRadians(42))//-42
                .forward(9.6)//og8.7

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    arm.SlightLower();
                })


                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(0);
                    arm.Five();
                })


                .waitSeconds(2)
                .back(8)
                .turn(Math.toRadians(48))//-48
                .strafeRight(1)
                .forward(47.5)


                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(1);
                    arm.High();
                })


                .waitSeconds(1)
                .back(10)
                .turn(Math.toRadians(-90))//90
                .strafeRight(29)//left
                .forward(3)//og 3

                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    claw.setPosition(1);
                    arm.SlightLower();
                })


                //CHANGE TO LEFT OR RIGHT BASED ON DETECTION OR DONT STRAFE AT ALL
                .waitSeconds(2)
                .back(5)
                .strafeRight(12)
                .build();





        /** Wait for the ga
         * me to begin */
        telemetry.addData(">", "Press Play to start Autonomous");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        webcam.stopStreaming();
        if (isStopRequested()) return;



        ParkingZone position = pipeline.getLastPosition();


        if (position == ParkingZone.ZONE1) {
            started = true;
            telemetry.addData("","Parking Zone 1");
            telemetry.update();
            drive.followTrajectorySequence(location1);


        } else if (position == ParkingZone.ZONE2) {
            started = true;
            telemetry.addData("","Parking Zone 2");
            telemetry.update();
            drive.followTrajectorySequence(location2);

        } else if (position == ParkingZone.ZONE3) {
            started = true;
            telemetry.addData("","Parking Zone 3");
            telemetry.update();
            drive.followTrajectorySequence(location3);


        }
        else {
            started = true;
            telemetry.addData("","Failed to detect");
            telemetry.update();
            drive.followTrajectorySequence(location2);


        }

        while (opModeIsActive()) {

            if (toucher.isPressed())
            {
                armMotor.setTargetPosition(0);
            }


            if (started == true) {

                webcam.stopStreaming();

            }

        }

    }
}
