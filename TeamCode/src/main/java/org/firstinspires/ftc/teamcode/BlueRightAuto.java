
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.MarkerDetection;
import org.firstinspires.ftc.teamcode.subsystems.ParkingZone;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="BlueRightAuto", group="Autonomous")
public class BlueRightAuto extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(-35.25, 64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        //flipping
        TrajectorySequence location1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .strafeLeft(23.5)
                .lineTo(new Vector2d(-11.75, 42))
                .turn(Math.toRadians(-33))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //Arm raises
                    arm.Raise();
                })
                .waitSeconds(3)
                .forward(12)//was 12
                //change 1 to appropriate distance based on tuning
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(1);
                    claw2.setPosition(0);
                })
                .waitSeconds(1.5)
                .back(12)
                .turn(Math.toRadians(33))
                .forward(12)
                .splineToSplineHeading(new Pose2d(-35.25, 11.75, Math.toRadians(180)), Math.toRadians(180))
                .turn(Math.toRadians(-90))
                //CHANGE TO LEFT OR RIGHT BASED ON DETECTION OR DONT STRAFE AT ALL
                .strafeRight(20.5)
                .build();

        TrajectorySequence location2 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .strafeLeft(23.5)
                .lineTo(new Vector2d(-11.75, 42))
                .turn(Math.toRadians(-33))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //Arm raises
                    arm.Raise();
                })
                .waitSeconds(3)
                .forward(12)//was 12
                //change 1 to appropriate distance based on tuning
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(1);
                    claw2.setPosition(0);
                })
                .waitSeconds(1.5)
                .back(12)
                .turn(Math.toRadians(33))
                .forward(12)
                .splineToSplineHeading(new Pose2d(-35.25, 11.75, Math.toRadians(180)), Math.toRadians(180))
                .turn(Math.toRadians(-90))
                //CHANGE TO LEFT OR RIGHT BASED ON DETECTION OR DONT STRAFE AT ALL
                .strafeLeft(3)
                .build();

        TrajectorySequence location3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .strafeLeft(23.5)
                .lineTo(new Vector2d(-11.75, 42))
                .turn(Math.toRadians(-33))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //Arm raises
                    arm.Raise();
                })
                .waitSeconds(3)
                .forward(12)//was 12
                //change 1 to appropriate distance based on tuning
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(1);
                    claw2.setPosition(0);
                })
                .waitSeconds(1.5)
                .back(12)
                .turn(Math.toRadians(33))
                .forward(12)
                .splineToSplineHeading(new Pose2d(-35.25, 11.75, Math.toRadians(180)), Math.toRadians(180))
                .turn(Math.toRadians(-90))
                //CHANGE TO LEFT OR RIGHT BASED ON DETECTION OR DONT STRAFE AT ALL
                .strafeLeft(23.5)
                .build();




        /** Wait for the ga
         * me to begin */
        telemetry.addData(">", "Press Play to start Autonomous");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        webcam.stopStreaming();
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
