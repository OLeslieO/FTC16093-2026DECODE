package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class MecanumDriveCommand extends CommandBase {

    private final MecanumDrive drive;
    private final GamepadEx gamepadEx;

    public MecanumDriveCommand(MecanumDrive drive, GamepadEx gamepadEx) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double y = -gamepadEx.getLeftY();
        double x = gamepadEx.getLeftX() * 1.1;
        double rx = gamepadEx.getRightX();

        drive.drive(y, x, rx);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
