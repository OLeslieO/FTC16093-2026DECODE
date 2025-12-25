package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.driving.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.commands.PreLimitCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(group = "0-competition", name = "TeleOp Solo")
public class TeleOpSolo extends CommandOpModeEx {
    GamepadEx gamepadEx1, gamepadEx2;
    NewMecanumDrive driveCore;
    PreLimitCommand preLimitCommand;
    Shooter shooter;
    Intake intake;


    private boolean isFieldCentric=false;
    public boolean isLimitOn = false;

    private boolean isAutoConveted = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        driveCore = new NewMecanumDrive(hardwareMap);

        TeleOpDriveCommand driveCommand = new TeleOpDriveCommand(driveCore,
                ()->gamepadEx1.getLeftX(),
                ()->gamepadEx1.getLeftY(),
                ()->gamepadEx1.getRightX(),
                ()->(gamepadEx1.getButton(GamepadKeys.Button.START) && !gamepad1.touchpad),
                ()->(gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)),
                ()->(isFieldCentric));

        intake = new Intake(hardwareMap);
//        frontArm.setLED(false);
        shooter = new Shooter(hardwareMap);
        preLimitCommand = new PreLimitCommand(intake,
                ()->(isLimitOn));



        if (Constants.Position.autoFinished) {
            driveCore.setPoseFromAuto(
                    Constants.Position.x,
                    Constants.Position.y,
                    Constants.Position.heading
            );
            Constants.Position.autoFinished = false;
            isAutoConveted = true;
        } else {
            driveCore.init(); // 纯 TeleOp
        }




        driveCore.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CommandScheduler.getInstance().schedule(driveCommand);
        CommandScheduler.getInstance().schedule(preLimitCommand);

        //timers
        new ButtonEx(()->getRuntime()>30).whenPressed(()->gamepad1.rumble(500));
        new ButtonEx(()->getRuntime()>60).whenPressed(()->gamepad1.rumble(500));
        new ButtonEx(()->getRuntime()>110).whenPressed(()->gamepad1.rumble(1000));
    }

    @Override
    public void onStart() {
        resetRuntime();
        shooter.accelerate_slow();



    }

    @Override
    public void functionalButtons() {

        //leftBumper -- intake
        //rightTrigger -- Shooter
        //leftTrigger -- preShooter
        //a -- preShooter & intake 反转

        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.BACK))
                .whenPressed(new InstantCommand(()->isFieldCentric=!isFieldCentric));
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.A))
                .whenPressed(new InstantCommand(()->isLimitOn=!isLimitOn));



        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(new InstantCommand(()->intake.intake()))
                .whenReleased(new InstantCommand(()->intake.init()));

        new ButtonEx(()->gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.5)
                .whenPressed(new InstantCommand(()->shooter.accelerate_mid()))
                .whenReleased(new InstantCommand(()->shooter.accelerate_slow()));



        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.Y))
                .whenPressed(new InstantCommand(()->shooter.accelerate_slow()))
                .whenReleased(new InstantCommand(()->shooter.accelerate_slow()));

        new ButtonEx(()->Math.abs(gamepadEx1.getRightY())>0.7)
                .whenPressed(new InstantCommand(()->shooter.accelerate_fast()))
                .whenReleased(new InstantCommand(()->shooter.accelerate_slow()));

        new ButtonEx(()->gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5)
                .whenPressed(new InstantCommand(()->shooter.shoot()))
                .whenReleased(new InstantCommand(()->shooter.init()));

        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(new ParallelCommandGroup(
                        new InstantCommand(()->intake.outtake()),
                        new InstantCommand(()->shooter.outtake())))
                .whenReleased(new ParallelCommandGroup(
                        new InstantCommand(()->intake.init()),
                        new InstantCommand(()->shooter.init())));

        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(new InstantCommand(()->shooter.emergency()))
                .whenReleased(new InstantCommand(()->shooter.stopAccelerate()));

        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP))
                .whenPressed(new InstantCommand(()->shooter.stopAccelerate()));


    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        telemetry.addData("shooter velocity", shooter.shooterDown.getVelocity());
        telemetry.addData("Heading", Math.toDegrees(driveCore.getHeading()));
        if(isFieldCentric) telemetry.addLine("Field Centric");
        else telemetry.addLine("Robot Centric");
        if(isLimitOn) telemetry.addLine("Limit On");
        else telemetry.addLine("Limit Off");
        Pose2D pose = driveCore.getPose();

        telemetry.addData("X (in)", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)",
                Math.toDegrees(pose.getHeading(AngleUnit.RADIANS)));
        telemetry.addData("isAutoConveted?",isAutoConveted);



        telemetry.update();
        telemetry.update();
    }
}