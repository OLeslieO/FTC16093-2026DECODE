package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.gamepad.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;

@TeleOp(name = "TeleOpDrive", group = "Main")
public class TeleOpDrive extends CommandOpMode {

    private MecanumDrive mecanumDrive;
    private MecanumDriveCommand mecanumDriveCommand;
    private Intake intake;
    private IntakeCommand intakeCommand;
    private GamepadEx gamepadEx1;
    private State state;

    public enum State {  }




    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        mecanumDrive = new MecanumDrive(telemetry, hardwareMap);
        intake = new Intake();

        mecanumDriveCommand = new MecanumDriveCommand(mecanumDrive);
        intakeCommand = new IntakeCommand(intake);

        mecanumDrive.setDefaultCommand(mecanumDriveCommand);
        intake.setDefaultCommand(intakeCommand);
    }

}
