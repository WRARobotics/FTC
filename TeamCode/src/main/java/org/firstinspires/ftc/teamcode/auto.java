package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

import java.util.ArrayList;

@Autonomous(name="auto")
public class auto extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        Drive drive=new Drive(hardwareMap);
        waitForStart();
        TrajectorySequence trajSeqLeft=drive.trajectorySequenceBuilder(new Pose2d(-36,60,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-60,-60,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-11,-60,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-11,-45,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-11,-60,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-50,-60,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-60,-35,Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(trajSeqLeft);
    }
}