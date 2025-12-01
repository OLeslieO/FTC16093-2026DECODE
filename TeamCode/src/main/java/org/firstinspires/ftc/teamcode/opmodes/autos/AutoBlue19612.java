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

// 我们在远端初始化，等到他们近端全射完到近端射预制 --> leave line

@Autonomous(name = "19612 Blue Far", group = "Competition")
public class AutoBlue19612 extends AutoOpModeEx {
    private FollowerEx follower;
    private AutoCommand autoCommand;
    private List<Command> actions;
    private Shooter shooter;
    private Intake intake;
    private Boolean actionRunning;


    private PathChainList pathChainList;

    private final Pose startPose = new Pose(1.939, -52.423, Math.toRadians(23.17));

    private final Pose scorePose = new Pose(100.644, -28.950, Math.toRadians(45));

    private final Pose intake1Pose3 = new Pose(-4.086, 3.046, Math.toRadians(180));
    private final Pose parkPose = new Pose(28.729, -49.009, Math.toRadians(11.24));
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
        PathChain score;
        PathChain park;

        score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(getCurrentPoint(), new Point(parkPose)))
                .setLinearHeadingInterpolation(getCurrentHeading(), parkPose.getHeading())
                .build();

        pathChainList.addPath(null, null, null,
                score, null,
                park);
    }

    @NonNull
    private Command actionEnd(){
        return new InstantCommand(()->this.actionRunning = false);
    }

    private void buildActions(){
        Command intakeCommand, accelerateCommand, scoreCommand, stopCommand, waitCommand, parkCommand;
        scoreCommand = autoCommand.shoot().andThen(actionEnd());
        intakeCommand = autoCommand.intake().andThen(actionEnd());
        accelerateCommand = autoCommand.accelerate().andThen(new WaitCommand(500).andThen(actionEnd()));
        waitCommand = new WaitCommand(20*1000).andThen(actionEnd());
        parkCommand = autoCommand.park();

        actions.addAll(Arrays.asList(waitCommand, accelerateCommand, intakeCommand,
                null, scoreCommand,
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