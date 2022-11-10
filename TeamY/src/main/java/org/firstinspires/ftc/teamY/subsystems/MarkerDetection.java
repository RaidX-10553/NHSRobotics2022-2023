package org.firstinspires.ftc.teamY.subsystems;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

//@Config
public class MarkerDetection extends AprilTagDetectionPipeline {

    public static int firstTag = 0;
    public static int secondTag = 1;
    public static int thirdTag = 2;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;


    private ParkingZone position = ParkingZone.UNKNOWN;
    private final Object positionLock = new Object();

    int numFramesWithoutDetection = 0;

    Telemetry telemetry;

    public MarkerDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public MarkerDetection() {
        this(null);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat output = super.processFrame(input);

        synchronized (positionLock) {
            List<AprilTagDetection> detections = getDetectionsUpdate();
            if(detections != null) {
                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        setDecimation(DECIMATION_LOW);
                    }
                } else {
                    numFramesWithoutDetection = 0;

                    for (AprilTagDetection detection : detections) {
                        // If the target is within 1 meter, turn on high decimation to
                        // increase the frame rate
                        if(detection.pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                            setDecimation(DECIMATION_HIGH);
                        }



                        if (detection.id == firstTag || detection.id == secondTag || detection.id == thirdTag) {
                            
                            if (detection.id == firstTag) {
                                position = ParkingZone.ZONE1;
                            } else if (detection.id == thirdTag) {
                                position = ParkingZone.ZONE3;
                            } else {
                                position = ParkingZone.ZONE2;
                            }

                            
                            break;
                        }
                    }
                }
            }
        }

        if(telemetry != null) {
            telemetry.addData("Position", position);
            telemetry.update();
        }

        return output;
    }

    public ParkingZone getLastPosition() {
        synchronized(positionLock) {
            return position;
        }
    }

}