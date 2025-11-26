package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.driving.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.commands.autos.driveAutoCommand;
import org.firstinspires.ftc.teamcode.utils.FollowerEx;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AutoBlue")
public class AutoBlue extends AutoOpModeEx {

    Shooter shooter;
    Intake intake;
    private Follower follower;

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;






        public Command runAutoCommand(){

            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 120.000), new Pose(44.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 84.000), new Pose(20.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 84.000), new Pose(60.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.000, 84.000), new Pose(44.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 60.000), new Pose(20.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 60.000), new Pose(60.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.000, 84.000), new Pose(44.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 36.000), new Pose(20.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 36.000), new Pose(64.000, 16.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                    .build();


            return new SequentialCommandGroup(
                    new ParallelRaceGroup(
                            new WaitCommand(4000),
                            new InstantCommand(() -> shooter.shortShoot())
                    ),

                    new driveAutoCommand(follower,Path1),
                    /*
                    new ParallelRaceGroup(
                            new driveAutoCommand(follower,Path1),
                            new InstantCommand(()-> shooter.acc())
                    ),

                     */
                    new ParallelRaceGroup(
                            new driveAutoCommand(follower,Path2),
                            new InstantCommand((()-> intake.intake()))
                    ),
                    new driveAutoCommand(follower,Path3),
                    /*
                    new ParallelRaceGroup(
                            new driveAutoCommand(follower,Path3),
                            new InstantCommand(()-> shooter.acc())
                    ),

                     */
                    new ParallelRaceGroup(
                            new WaitCommand(4000),
                            new InstantCommand(() -> shooter.midShoot())),
                    new driveAutoCommand(follower,Path4),
                    new ParallelRaceGroup(
                            new driveAutoCommand(follower,Path5),
                            new InstantCommand(()-> intake.intake())
                    ),
                    new driveAutoCommand(follower,Path6),
                    /*
                    new ParallelRaceGroup(
                            new driveAutoCommand(follower,Path6),
                            new InstantCommand(()-> shooter.acc())
                    ),

                     */
                    new ParallelRaceGroup(
                            new WaitCommand(4000),
                            new InstantCommand(() -> shooter.midShoot())
                    ),
                    new driveAutoCommand(follower,Path7),
                    new ParallelRaceGroup(
                            new driveAutoCommand(follower,Path8),
                            new InstantCommand(()-> intake.intake())
                    ),
                    new driveAutoCommand(follower,Path9),
                    /*
                    new ParallelRaceGroup(
                            new driveAutoCommand(follower,Path9),
                            new InstantCommand(()-> shooter.acc())
                    ),

                     */
                    new ParallelRaceGroup(
                            new WaitCommand(5000),
                            new InstantCommand(() -> shooter.longShoot())
                    )


            );
    }



    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new FollowerEx(hardwareMap, FConstants.class, LConstants.class);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        CommandScheduler.getInstance().schedule(runAutoCommand());
    }
    private void periodic() {
        CommandScheduler.getInstance().run();

        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());


        telemetry.update();
    }

    @Override
    public void run() {
        periodic();
    }
}



