package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.driving.NewMecanumDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class PreLimitCommand extends CommandBase {

    private final BooleanSupplier isLimitOn;

    private final Intake intake;

    public PreLimitCommand(
            Intake intake,

            BooleanSupplier isLimitOn) {
        this.intake = intake;

        this.isLimitOn = isLimitOn;
    }

    @Override
    public void execute() {

        if (isLimitOn.getAsBoolean()) {
           intake.limitOn();
        } else {
            intake.limitOff();
        }

    }
}
