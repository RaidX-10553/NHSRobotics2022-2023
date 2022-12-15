
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.ParkingZone;
import org.firstinspires.ftc.teamcode.subsystems.MarkerDetection;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.subsystems.Arm;


@Autonomous(name="BlueLeftAuto2", group="Autonomous")
public class BlueLeftAuto extends LinearOpMode {

    /* Declare OpMode members. */

    OpenCvWebcam webcam;

    MarkerDetection pipeline;

    boolean started = false;

    Servo claw1;
    Servo claw2;

    DcMotorEx armMotor;
    Arm arm;

    @Override
    public void runOpMode() {
        pipeline = new MarkerDetection();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //Still working on the trajectories, not final
        //Road Runner Trajectory


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        arm = new Arm(armMotor);

        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Pose2d startPose = new Pose2d(35.25, 64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        //flipping
        TrajectorySequence location1 = drive.trajectorySequenceBuilder(startPose)

                .strafeRight(23.5)
                .lineTo(new Vector2d(11.75, 35.25))
                .turn(Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    //Arm raises
                    arm.Raise();
                })
                //change 1 to appropriate distance based on tuning
                .forward(3)
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    //Claw Opens
                    claw1.setPosition(1);
                    claw2.setPosition(0);
                })
                .back(3)
                .turn(Math.toRadians(-45))
                .forward(10)
                .splineToSplineHeading(new Pose2d(35.25, 11.75, Math.toRadians(0)), Math.toRadians(0))
                .turn(Math.toRadians(90))
                //CHANGE TO LEFT OR RIGHT BASED ON DETECTION OR DONT STRAFE AT ALL
                .strafeRight(23.5)
                .build();

        TrajectorySequence location2 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    //claw close

                })
                .strafeRight(23.5)
                .lineTo(new Vector2d(11.75, 35.25))
                .turn(Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //Arm raises
                    arm.Raise();
                })

                //change 1 to appropriate distance based on tuning


                .addDisplacementMarker(() -> {
                    //Claw Opens
                    claw1.setPosition(1);
                    claw2.setPosition(0);
                })

                .turn(Math.toRadians(-45))
                .forward(10)
                .splineToSplineHeading(new Pose2d(35.25, 11.75, Math.toRadians(0)), Math.toRadians(0))
                .turn(Math.toRadians(90))
                //CHANGE TO LEFT OR RIGHT BASED ON DETECTION OR DONT STRAFE AT ALL

                .build();

        TrajectorySequence location3 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    //claw close

                })
                .strafeRight(23.5)
                .lineTo(new Vector2d(11.75, 35.25))
                .turn(Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    //Arm raises
                    arm.Raise();
                })

                //change forw/back val to appropriate distance based on tuning

                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    //Claw Opens
                    claw1.setPosition(1);
                    claw2.setPosition(0);
                })

                .turn(Math.toRadians(-45))
                .forward(10)
                .splineToSplineHeading(new Pose2d(35.25, 11.75, Math.toRadians(0)), Math.toRadians(0))
                .turn(Math.toRadians(90))
                //CHANGE TO LEFT OR RIGHT BASED ON DETECTION OR DONT STRAFE AT ALL
                .strafeLeft(23.5)
                .build();




        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start Autonomous");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        if (isStopRequested()) return;


        claw1.setPosition(0.5);
        claw2.setPosition(0.5);

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

            if (started == true) {

                webcam.stopStreaming();

            }

        }
    }
}
