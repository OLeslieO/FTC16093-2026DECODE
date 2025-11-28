package org.firstinspires.ftc.teamcode.opmodes.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

@Autonomous(name = "Red Far Auto", group = "Competition")
public class AutoFarRed extends AutoOpModeEx {
    private FollowerEx follower;
    private AutoCommand autoCommand;
    private List<Command> actions;
    private Shooter shooter;
    private Intake intake;
    private Boolean actionRunning;


    private PathChainList pathChainList;

    private final Pose startPose = new Pose(1.939, 52.423, Math.toRadians(-23.17));

    private final Pose scorePose = new Pose(1.939, 52.423, Math.toRadians(-23.17));

    private final Pose prepare1Pose = new Pose(4.137, 7.539, Math.toRadians(-90));
    private final Pose intake1Pose1 = new Pose(8.091, 2.056, Math.toRadians(-148));
    private final Pose intake1Pose2 = new Pose(5.565, 0.287, Math.toRadians(-180));
    private final Pose intake1Pose3 = new Pose(-4.086, 0.046, Math.toRadians(-180));
    private final Pose prepare2Pose  = new Pose(31.000, 32.834, Math.toRadians(-90));
    private final Pose intake2Pose1 = new Pose(31.000, 29.932, Math.toRadians(-90));
    private final Pose intake2Pose2 = new Pose(31.000, 23.932, Math.toRadians(-90));
    private final Pose intake2Pose3 = new Pose(31.000, -3, Math.toRadians(-90));
    private final Pose parkPose = new Pose(30.729, 49.009, Math.toRadians(-11.24));
    private int currentPathId = 0;


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
        PathChain prepare1, intake1_1, intake1_2, intake1_3, after1,
                prepare2, intake2_1, intake2_2, intake2_3, after2,
                score1, score2;
        PathChain park;


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

//        after1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(intake1Pose), new Point(prepareGatePose)))
//                .setLinearHeadingInterpolation(intake1Pose.getHeading(), prepareGatePose.getHeading())
//                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose3), new Point(scorePose)))
                .setLinearHeadingInterpolation(intake1Pose3.getHeading(), scorePose.getHeading())
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

//        after2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(intake2Pose3), new Point(prepare2Pose)))
//                .setLinearHeadingInterpolation(intake2Pose3.getHeading(), prepare2Pose.getHeading())
//                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose3), new Point(scorePose)))
                .setLinearHeadingInterpolation(intake2Pose3.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(getCurrentPoint(), new Point(parkPose)))
                .setLinearHeadingInterpolation(getCurrentHeading(), parkPose.getHeading())
                .build();

        pathChainList.addPath(null, null, null, null,
                prepare1, intake1_1, intake1_2, intake1_3,
                score1, null,
                prepare2, intake2_1, intake2_2, intake2_3, //不拿远端那组球就注释掉
//                score2, null, //最后一组如果不射就注释掉
                park);
    }

    @NonNull
    private Command actionEnd(){
        return new InstantCommand(()->this.actionRunning = false);
    }

    private void buildActions(){
        Command intakeCommand, accelerateFastCommand, scoreFarCommand, stopCommand, waitCommand, parkCommand;
        scoreFarCommand = autoCommand.shootFar().andThen(actionEnd());
        intakeCommand = autoCommand.intake().andThen(actionEnd());
        accelerateFastCommand = autoCommand.accelerateFast().andThen(actionEnd());
//        openGateCommand = new WaitCommand(600).andThen(actionEnd());
        waitCommand = new WaitCommand(2000).andThen(actionEnd());
        stopCommand = autoCommand.stopAll();
        parkCommand = autoCommand.park();

        actions.addAll(Arrays.asList(accelerateFastCommand, waitCommand, intakeCommand, scoreFarCommand,
                null, null, null, null,
                null, scoreFarCommand,
                null, null, null, null,  //不拿远端那组球就注释掉`
//                null, scoreFarCommand, //最后一组如果不射就注释掉
                parkCommand));
    }

    private void periodic() {
        CommandScheduler.getInstance().run();
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("drive error",follower.driveError);
        telemetry.addData("follower finished",follower.isFinished);
        telemetry.addData("action finished", !this.actionRunning);
        telemetry.addData("current path /id", currentPathId);
        telemetry.addData("Actions size", actions.size());
        telemetry.addData("PathChainList size", pathChainList.size());
        telemetry.update();
    }

    @Override
    public void run() {
        if(actions.size() != pathChainList.size()){
            throw new IllegalStateException(
                    "Actions count (" + actions.size() +
                            ") does not match path count (" + pathChainList.size() + ")"
            );
        }
        Iterator<PathChain> it = pathChainList.iterator();
        int pathCount = 0;
        while (it.hasNext()){
            pathCount+=1;
            if (!opModeIsActive())break;
            periodic();
            if(!follower.isBusy() && !this.actionRunning){
                PathChain path = it.next();
                if(path!=null){
//                    if (pathCount == 9 || pathCount == 10 || pathCount == 11 || pathCount == 13
//                            || pathCount == 17 || pathCount == 18
//                            || pathCount == 20
//                            || pathCount == 24 || pathCount == 25){
//                        follower.followPath(path,0.2,false);
//                    }
//                    else {
//                        follower.followPath(path, 1,true);
//                    }
                    follower.followPath(path, 0.6,true);
                }

                Command currentAction = actions.get(currentPathId);
                if(currentAction!=null){
                    currentAction.schedule();
                    this.actionRunning = true;
                }
                currentPathId++;
            }
        }
    }
}