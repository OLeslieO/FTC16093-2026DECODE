package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeCommand extends CommandBase {

    private final Intake intake;
    private final GamepadEx gamepadEx;

    public IntakeCommand(Intake intake, GamepadEx gamepadEx) {
        this.intake = intake;
        this.gamepadEx = gamepadEx;
        addRequirements(intake);
    }

    @Override
    public void execute() {

        if (gamepadEx.getButton(GamepadKeys.Button.X)) {
            intake.intakeIn();
        } else if (gamepadEx.getButton(GamepadKeys.Button.Y)) {
            intake.intakeOut();
        } else {
            intake.stop();
        }
    }
}
