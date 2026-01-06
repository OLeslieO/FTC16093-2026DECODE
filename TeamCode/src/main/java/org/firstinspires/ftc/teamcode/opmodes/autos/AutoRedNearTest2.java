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

@Autonomous(name = "Red Near Auto Test 2b", group = "Competition")
public class AutoRedNearTest2 extends AutoOpModeEx {
    private FollowerEx follower;
    private AutoCommand autoCommand;
    private List<Command> actions;
    private Shooter shooter;

    private Intake intake;
    private Boolean actionRunning;


    private PathChainList pathChainList;

    private final Pose startPose = new Pose(51.113, -47.224, Math.toRadians(309));

    private final Pose scorePose = new Pose(31.551, -31.884, Math.toRadians(317));

    private final Pose scoreMidPose = new Pose(11.416, -10.589, Math.toRadians(315));

    private final Pose prepare1Pose = new Pose(12.439, -27.770, Math.toRadians(270));

    private final Pose intake1Pose3 = new Pose(9.439, -53.656, Math.toRadians(270));
    private final Pose prepareGatePose = new Pose(1.635, -48.42, Math.toRadians(270));
    private final Pose openGatePose = new Pose(-5.635, -53.905, Math.toRadians(270));

    private final Pose openGateIntakePose = new Pose(-13.705,-61,Math.toRadians(312.54));
    private final Pose openGateIntakePose2 = new Pose(-25.616,-60.493,Math.toRadians(316));

    private final Pose prepare2Pose = new Pose(-13.4, -29.18, Math.toRadians(270));
    private final Pose intake2Pose1 = new Pose(53.764, 29.378, Math.toRadians(-88));
    private final Pose intake2Pose2 = new Pose(53.764, 20.378, Math.toRadians(-88));
    private final Pose intake2Pose3 = new Pose(-15, -58, Math.toRadians(268));
    private final Pose prepare3Pose = new Pose(-36.52, -28.23, Math.toRadians(270));

    private final Pose intake3Pose3 = new Pose(-36.52, -58, Math.toRadians(270));
    private final Pose intakeLoad1 = new Pose(-42.991, -57, Math.toRadians(180));
    private final Pose intakeLoad3 = new Pose(-62, -57, Math.toRadians(180));
    private final Pose parkPose = new Pose(9.439, -27, Math.toRadians(270));

    private final Pose openGatePose2 = new Pose(72.982, 7.239, Math.toRadians(-1));
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

        follower.setMaxPower(1);
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
                prepare1,  after1,
                prepare2, after2,
                prepare3,  after3,
                score1, score2, score3,
                openGate, afterOpenGate,openGate2,
                intake1,intake2,intake3,
                openGateIntake,score4,intakeLoad,scoreMidLoad,prepareMid,
                intakeGate,preGate;
        PathChain park;

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        prepare1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(prepare1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepare1Pose.getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare1Pose), new Point(intake1Pose3)))
                .setLinearHeadingInterpolation(prepare1Pose.getHeading(), intake1Pose3.getHeading())
                .build();



        after1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose3), new Point(prepare1Pose)))
                .setLinearHeadingInterpolation(intake1Pose3.getHeading(), prepare1Pose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare2Pose), new Point(openGatePose)))
                .setLinearHeadingInterpolation(prepare2Pose.getHeading(), openGatePose.getHeading())
                .build();

        openGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(prepareGatePose), new Point(openGatePose2)))
                .setLinearHeadingInterpolation(prepareGatePose.getHeading(), openGatePose2.getHeading())
                .build();

        openGateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(prepare2Pose), new Point(openGateIntakePose)))
                .setLinearHeadingInterpolation(prepare2Pose.getHeading(), openGateIntakePose.getHeading())
                .build();
        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(openGateIntakePose2), new Point(scorePose)))
                .setLinearHeadingInterpolation(openGateIntakePose2.getHeading(), scorePose.getHeading())
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

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare2Pose), new Point(intake2Pose3)))
                .setLinearHeadingInterpolation(prepare2Pose.getHeading(), intake2Pose3.getHeading())
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

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare3Pose), new Point(intake3Pose3)))
                .setLinearHeadingInterpolation(prepare3Pose.getHeading(), intake3Pose3.getHeading())
                .build();



        after3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose3), new Point(prepare3Pose)))
                .setLinearHeadingInterpolation(intake3Pose3.getHeading(), prepare3Pose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(prepare3Pose.getHeading(), scorePose.getHeading())
                .build();

        prepareMid = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(intakeLoad1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeLoad1.getHeading())
                .build();

        intakeLoad = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeLoad1), new Point(intakeLoad3)))
                .setLinearHeadingInterpolation(intakeLoad1.getHeading(), intakeLoad3.getHeading())
                .build();

        scoreMidLoad = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeLoad3), new Point(scoreMidPose)))
                .setLinearHeadingInterpolation(intakeLoad3.getHeading(), scoreMidPose.getHeading())
                .build();
        intakeGate = follower.pathBuilder()
                .addPath(new BezierLine(new Point(openGateIntakePose), new Point(openGateIntakePose2)))
                .setLinearHeadingInterpolation(openGateIntakePose.getHeading(), openGateIntakePose2.getHeading())
                .build();

        preGate = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(prepare2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepare2Pose.getHeading())
                .build();


        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreMidPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scoreMidPose.getHeading(), parkPose.getHeading())
                .build();

        pathChainList.addPath(scorePreload, null, null, null,null,
                prepare2,intake2,
                after2,score2,null,
                preGate,openGateIntake,intakeGate,null,
                score4,null,
                preGate,openGateIntake,intakeGate,null,
                score4,null,
                prepare1,intake1,
                after1,score1,null,
                prepare3,intake3,
                after3,score3,null,park);
    }

    @NonNull
    private Command actionEnd(){
        return new InstantCommand(()->this.actionRunning = false);
    }

    private void buildActions(){
        Command intakeCommand, accelerateCommand, scoreCommand, openGateCommand, waitCommand,
                scoreMidCommand, accelerateMidCommand, preLimitCommand;
        scoreCommand = autoCommand.shoot().andThen(actionEnd());
        scoreMidCommand = autoCommand.shootMid().andThen(actionEnd());
        intakeCommand = autoCommand.intake().andThen(actionEnd());
        accelerateCommand = autoCommand.accelerate().andThen(actionEnd());
        accelerateMidCommand = autoCommand.accelerateMid().andThen(actionEnd());
        openGateCommand = new WaitCommand(700).andThen(actionEnd());
        waitCommand = new WaitCommand(450).andThen(actionEnd());
        preLimitCommand = autoCommand.preLimitOn().andThen(actionEnd());

        actions.addAll(Arrays.asList(accelerateCommand,preLimitCommand, waitCommand, intakeCommand, scoreCommand,
                null, null,
                null, null, scoreCommand,
                null,null, null, openGateCommand,
                null, scoreCommand,
                null,null, null, openGateCommand,
                null, scoreCommand,
                null, null,
                null,null,scoreCommand,
                null,null,
                null, null,scoreMidCommand, new WaitCommand(99999)));
    }

    private void periodic() {
        CommandScheduler.getInstance().run();
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooterUpVelocity", shooter.shooterUp.getVelocity());
        telemetry.addData("shooterDownVelocity", shooter.shooterDown.getVelocity());
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
                    follower.followPath(path, 1,true);
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