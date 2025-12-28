package org.firstinspires.ftc.teamcode.opmodes.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.FollowerEx;
import org.firstinspires.ftc.teamcode.utils.PathChainList;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Blue Near Auto", group = "Competition")
public class AutoNearBlue extends AutoOpModeEx {
    private FollowerEx follower;
    private AutoCommand autoCommand;
    private List<Command> actions;
    private Shooter shooter;
    private Intake intake;
    private Boolean actionRunning;


    private PathChainList pathChainList;

    private final Pose startPose = new Pose(123.850, -21.490, Math.toRadians(46     ));

    private final Pose scorePose = new Pose(105.644, -32.950, Math.toRadians(49));

    private final Pose prepare1Pose = new Pose(79.249, -38.147, Math.toRadians(90));
    private final Pose intake1Pose1 = new Pose(79.249, -28.239, Math.toRadians(90));
    private final Pose intake1Pose2 = new Pose(79.249, -20.239, Math.toRadians(90));
    private final Pose intake1Pose3 = new Pose(79.249, -8.239, Math.toRadians(90));
    private final Pose prepareGatePose = new Pose(79.249, -20.639, Math.toRadians(90));
    private final Pose openGatePose = new Pose(67.982, -7.239, Math.toRadians(90));
    private final Pose openGatePose2 = new Pose(70.982, -10.239, Math.toRadians(0));
    private final Pose prepare2Pose = new Pose(53.050, -35.834, Math.toRadians(90));
    private final Pose intake2Pose1 = new Pose(53.764, -29.978, Math.toRadians(90));
    private final Pose intake2Pose2 = new Pose(53.764, -21.378, Math.toRadians(90));
    private final Pose intake2Pose3 = new Pose(53.764, 0, Math.toRadians(90));
    private final Pose prepare3Pose = new Pose(30.000, -36.730, Math.toRadians(90));
    private final Pose intake3Pose1 = new Pose(30.000, -29.932, Math.toRadians(90));
    private final Pose intake3Pose2 = new Pose(30.000, -23.932, Math.toRadians(90));
    private final Pose intake3Pose3 = new Pose(30.000, 3, Math.toRadians(90));
    private final Pose parkPose = new Pose(67.982, -20.000, Math.toRadians(90));
    private int currentPathId = 0;


    private boolean poseSaved = false;


    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new FollowerEx(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        this.pathChainList = new PathChainList();
        this.actions = new ArrayList<>();
        this.autoCommand = new AutoCommand(shooter, intake);
        this.actionRunning = false;

        buildPaths();
        buildActions();

        follower.setMaxPower(0.8);
    }

    @NonNull
    private Point getCurrentPoint(){
        return new Point(follower.getPose().getX(),follower.getPose().getY());
    }

    private double getCurrentHeading(){
        return follower.getPose().getHeading();
    }

    private void buildPaths() {
        PathChain scorePreload,
                prepare1, intake1_1, intake1_2, intake1_3, after1,
                prepare2, intake2_1, intake2_2, intake2_3, after2,
                prepare3, intake3_1, intake3_2, intake3_3, after3,
                score1, score2, score3, openGate, afterOpenGate;
        PathChain park;

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        prepare1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(prepare1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepare1Pose.getHeading())
                .build();

        intake1_1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare1Pose), new Point(intake1Pose1)))
                .setLinearHeadingInterpolation(prepare1Pose.getHeading(), intake1Pose1.getHeading())
                .build();
        intake1_2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose1), new Point(intake1Pose2)))
                .setLinearHeadingInterpolation(intake1Pose1.getHeading(), intake1Pose2.getHeading())
                .build();
        intake1_3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose2), new Point(intake1Pose3)))
                .setLinearHeadingInterpolation(intake1Pose2.getHeading(), intake1Pose3.getHeading())
                .build();

        after1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose3), new Point(prepareGatePose)))
                .setLinearHeadingInterpolation(intake1Pose3.getHeading(), prepareGatePose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepareGatePose), new Point(openGatePose)))
                .setLinearHeadingInterpolation(prepareGatePose.getHeading(), openGatePose.getHeading())
                .build();

//        afterOpenGate = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(openGatePose), new Point(prepare1Pose)))
//                .setLinearHeadingInterpolation(openGatePose.getHeading(), prepare1Pose.getHeading())
//                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(prepare1Pose.getHeading(), scorePose.getHeading())
                .build();

        prepare2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(prepare2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepare2Pose.getHeading())
                .build();

        intake2_1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare2Pose), new Point(intake2Pose1)))
                .setLinearHeadingInterpolation(prepare2Pose.getHeading(), intake2Pose1.getHeading())
                .build();
        intake2_2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose1), new Point(intake2Pose2)))
                .setLinearHeadingInterpolation(intake2Pose1.getHeading(), intake2Pose2.getHeading())
                .build();
        intake2_3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose2), new Point(intake2Pose3)))
                .setLinearHeadingInterpolation(intake2Pose2.getHeading(), intake2Pose3.getHeading())
                .build();

        after2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose3), new Point(prepare2Pose)))
                .setLinearHeadingInterpolation(intake2Pose3.getHeading(), prepare2Pose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(prepare2Pose.getHeading(), scorePose.getHeading())
                .build();

        prepare3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(prepare3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepare3Pose.getHeading())
                .build();

        intake3_1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare3Pose), new Point(intake3Pose1)))
                .setLinearHeadingInterpolation(prepare3Pose.getHeading(), intake3Pose1.getHeading())
                .build();
        intake3_2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose1), new Point(intake3Pose2)))
                .setLinearHeadingInterpolation(intake3Pose1.getHeading(), intake3Pose2.getHeading())
                .build();
        intake3_3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose2), new Point(intake3Pose3)))
                .setLinearHeadingInterpolation(intake3Pose2.getHeading(), intake3Pose3.getHeading())
                .build();

        after3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose3), new Point(prepare2Pose)))
                .setLinearHeadingInterpolation(intake3Pose3.getHeading(), prepare3Pose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(prepare3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

        pathChainList.addPath(scorePreload, null, null, null,
                prepare1, intake1_1, intake1_2, intake1_3,
                after1, openGate, null, score1, null,
                prepare2, intake2_1, intake2_2, intake2_3,
                after2, score2, null,
                prepare3, intake3_1, intake3_2, intake3_3,
                after3, score3, null, park);
    }

    @NonNull
    private Command actionEnd(){
        return new InstantCommand(()->this.actionRunning = false);
    }

    private void finalPosition(){
        Constants.Position.x = follower.getPose().getX();
        Constants.Position.y = follower.getPose().getY();
        Constants.Position.heading = follower.getPose().getHeading();
        Constants.Position.autoFinished = true;
    }


    private void buildActions(){
        Command intakeCommand, accelerateCommand, scoreCommand, openGateCommand, waitCommand;
        scoreCommand = autoCommand.shoot().andThen(actionEnd());
        intakeCommand = autoCommand.intake().andThen(actionEnd());
        accelerateCommand = autoCommand.accelerate().andThen(actionEnd());
        openGateCommand = new WaitCommand(600).andThen(actionEnd());
        waitCommand = new WaitCommand(450).andThen(actionEnd());

        actions.addAll(Arrays.asList(accelerateCommand, waitCommand, intakeCommand, scoreCommand,
                null, null, null, null,
                null, null, openGateCommand, null, scoreCommand,
                null, null, null, null,
                null, null, scoreCommand,
                null, null, null, null,
                null, null, scoreCommand, new WaitCommand(99999)));
    }

    private void periodic() {
        CommandScheduler.getInstance().run();
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("drive error",follower.driveError);
        telemetry.addData("shooterUpVelocity", shooter.shooterUp.getVelocity());
        telemetry.addData("follower finished",follower.isFinished);
        telemetry.addData("action finished", !this.actionRunning);
        telemetry.addData("current path /id", currentPathId);
        telemetry.addData("Actions size", actions.size());
        telemetry.addData("PathChainList size", pathChainList.size());
        telemetry.update();
    }

    @Override
    public void run() {
        if (actions.size() != pathChainList.size()) {
            throw new IllegalStateException(
                    "Actions count (" + actions.size() +
                            ") does not match path count (" + pathChainList.size() + ")"
            );
        }

        Iterator<PathChain> it = pathChainList.iterator();

        while (it.hasNext() && opModeIsActive()) {
            periodic();

            if (!follower.isBusy() && !actionRunning) {
                PathChain path = it.next();

                if (path != null) {
                    follower.followPath(path, 0.9, true);
                }

                Command currentAction = actions.get(currentPathId);
                if (currentAction != null) {
                    currentAction.schedule();
                    actionRunning = true;
                }

                currentPathId++;
            }
        }

        // Auto 正常结束，保存Position
        if (opModeIsActive() && !poseSaved) {
            finalPosition();
            poseSaved = true;
        }
    }

}