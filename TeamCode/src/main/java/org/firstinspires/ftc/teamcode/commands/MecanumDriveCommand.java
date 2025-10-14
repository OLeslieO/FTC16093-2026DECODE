package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;


public class MecanumDriveCommand extends CommandBase {
    private final MecanumDrive mecanumDrive;

    public MecanumDriveCommand(MecanumDrive mecanumDrive) {
        this.mecanumDrive = mecanumDrive;
        addRequirements(mecanumDrive);
    }


    @Override
    public void execute() {
        mecanumDrive.drive();

    }
}

