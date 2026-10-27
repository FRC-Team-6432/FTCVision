package org.firstinspires.ftc.teamcode;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Comparator;

@TeleOp
public class FollowCameraNew extends LinearOpMode {
    //TODO: define motors, sensors, processors here
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    private AprilTagPoseFtc myFtcPose;
    private final Pose3D targetPose = new Pose3D(
        new Position(DistanceUnit.MM, 0, 500, 0, System.nanoTime()),
        new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0, System.nanoTime())
    );

    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: add initialise code here
        initDrive();
        initVision();

        double forward = 0;
        double turn = 0;
        double strafe = 0;

        waitForStart();
        resetRuntime();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //TODO: add loop code here
                telemetry.addData("Runtime: ", getRuntime());
                telemetry.addLine();

                handleAprilTagDetections();

                if (gamepad1.a) {

                    ArrayList<Double> driveOutputs = goToTargetPose(targetPose);
                    forward = driveOutputs.get(0);
                    strafe = driveOutputs.get(1);
                    turn = driveOutputs.get(2);
                } else {
                    forward = gamepad1.left_stick_y;
                    strafe = gamepad1.left_stick_x;
                    turn = gamepad1.right_stick_x;
                }

                drive(forward, strafe, turn);

                telemetry.update();
                sleep(20);
            }
            this.visionPortal.close();
        }
    }

    private void initDrive() {
//        this.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        this.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        this.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        this.backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initVision() {
        AprilTagLibrary aprilTagLibrary = AprilTagGameDatabase.getBioBuzzTagLibrary();

        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .setCameraPose(
                        new Position(DistanceUnit.MM, 0, 0, 0, System.nanoTime()),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, System.nanoTime())
                        )
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(aprilTagLibrary)
                .build();

        this.visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(1280, 800))
                .enableLiveView(true)
                .addProcessor(this.aprilTagProcessor)
                .build();
    }

    private void handleAprilTagDetections() {
        ArrayList<AprilTagClusterDetection> validTags = getValidTags();

        if (validTags.isEmpty()) {
            myFtcPose = null;
            return;
        }

        validTags.sort(Comparator.comparing((AprilTagClusterDetection detection) -> detection.ftcPose.range));

        myFtcPose = validTags.get(0).ftcPose;

        telemetry.addData("Range: ", myFtcPose.range);
        telemetry.addData("Bearing: ", myFtcPose.bearing);
        telemetry.addData("Elevation: ", myFtcPose.elevation);
        telemetry.addData("Pitch: ", myFtcPose.pitch);
        telemetry.addData("Roll: ", myFtcPose.roll);
        telemetry.addData("Yaw: ", myFtcPose.yaw);
        telemetry.addData("X: ", myFtcPose.x);
        telemetry.addData("Y: ", myFtcPose.y);
        telemetry.addData("Z: ", myFtcPose.z);
    }

    private ArrayList<AprilTagClusterDetection> getValidTags() {
        ArrayList<AprilTagDetection> aprilTagDetections = this.aprilTagProcessor.getDetections();
        ArrayList<AprilTagClusterDetection> validTags = new ArrayList<>();

        for (AprilTagDetection detection : aprilTagDetections) {
            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }
            AprilTagClusterDetection clusterDetection = (AprilTagClusterDetection) detection;

            if (clusterDetection.ftcPose.roll > 90 || clusterDetection.ftcPose.roll < -90) {
                continue;
            }

            validTags.add(clusterDetection);
        }
        return validTags;
    }

    private void drive(double forward, double strafe, double rotation) {
        telemetry.addLine("------ DRIVE ------");
        telemetry.addData("Forward: ", forward);
        telemetry.addData("Strafe: ", strafe);
        telemetry.addData("Turn: ", rotation);
        telemetry.addLine();
//        frontLeft.setPower((forward + rotation) - strafe);
//        frontRight.setPower((forward - rotation) + strafe);
//        backRight.setPower((forward - rotation) - strafe);
//        backLeft.setPower((forward + rotation) + strafe);
    }

    private ArrayList<Double> goToTargetPose(Pose3D targetPose) {
        ArrayList<Double> driveOutputs = new ArrayList<>();
        if (this.myFtcPose == null || this.targetPose == null) {
            driveOutputs.add(0.0);
            driveOutputs.add(0.0);
            driveOutputs.add(0.0);
            return driveOutputs;
        }

        double gain = 0.001;

        double xError = this.myFtcPose.x - targetPose.getPosition().x;
        double yError = this.myFtcPose.y - targetPose.getPosition().y;

        double forward = yError * gain;
        double strafe = xError * gain;
        double turn = 0;

        driveOutputs.add(forward);
        driveOutputs.add(strafe);
        driveOutputs.add(turn);

        return driveOutputs;
    }
}