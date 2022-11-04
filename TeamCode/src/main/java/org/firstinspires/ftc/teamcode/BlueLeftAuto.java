
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ParkingZone;
import org.firstinspires.ftc.teamcode.subsystems.MarkerDetection;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="BlueLeftAuto2", group="Autonomous")
public class BlueLeftAuto extends LinearOpMode {

    /* Declare OpMode members. */

    OpenCvWebcam webcam;

    MarkerDetection pipeline;

    boolean started = false;

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


        Pose2d startPose = new Pose2d(36, 61, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        //flipping
        TrajectorySequence location1 = driveTrain.trajectorySequenceBuilder(startPose)
                .strafeLeft(32)
                .forward(32)
                .build();

        TrajectorySequence location2 = driveTrain.trajectorySequenceBuilder(startPose)
                .forward(32)
                .build();

        TrajectorySequence location3 = driveTrain.trajectorySequenceBuilder(startPose)
                .strafeRight(30)
                .forward(32)
                .build();




        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start Autonomous");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)

        waitForStart();

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

            if (started = true) {

                webcam.stopStreaming();

            }

        }
    }
}
