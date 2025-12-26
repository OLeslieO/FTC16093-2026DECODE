package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.driving.NewMecanumDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class PreLimitCommand extends CommandBase {

    private final BooleanSupplier isLimitOn;

    private final Shooter shooter;

    private final Intake intake;

    private final LED led;

    private boolean lastLimitOn = true;


    public PreLimitCommand(
            Shooter shooter,
            Intake intake,
            LED led,

            BooleanSupplier isLimitOn) {
        this.shooter = shooter;
        this.intake = intake;
        this.led = led;
        this.isLimitOn = isLimitOn;
    }

    @Override
    public void execute() {

        boolean limitOn = isLimitOn.getAsBoolean();

        // 状态发生变化
        if (limitOn != lastLimitOn) {

            if (!limitOn) {
                // ON -> OFF：立刻设默认 slow
                shooter.accelerate_slow();
            } else {
                // OFF -> ON：立刻 idle
                shooter.accelerate_idle();
            }

            lastLimitOn = limitOn;
        }

        // 持续行为
        if (limitOn) {
            intake.limitOn();
            led.setNone();
        } else {
            intake.limitOff();
            led.setBlue();
        }
    }

}
