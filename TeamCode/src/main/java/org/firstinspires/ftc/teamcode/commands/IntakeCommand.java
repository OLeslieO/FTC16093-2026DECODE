package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (gamepad1.xWasReleased()) {
            intake.autoIntake();
        } else if (gamepad1.yWasReleased()) {
            intake.manualIntake();
        } else {
            intake.allStop();

        }
    }

}
