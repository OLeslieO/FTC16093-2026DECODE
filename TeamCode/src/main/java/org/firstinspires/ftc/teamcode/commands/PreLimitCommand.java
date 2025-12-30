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

    private final BooleanSupplier isVelocityDetecting;

    private final Shooter shooter;

    private final Intake intake;

    private final LED led;

    private boolean lastLimitOn = false;



    private boolean redLedTriggered = false;


    public PreLimitCommand(
            Shooter shooter,
            Intake intake,
            LED led,

            BooleanSupplier isVelocityDetecting,

            BooleanSupplier isLimitOn) {
        this.shooter = shooter;
        this.intake = intake;
        this.led = led;
        this.isLimitOn = isLimitOn;
        this.isVelocityDetecting = isVelocityDetecting;
    }

    @Override
    public void execute() {

        boolean limitOn = isLimitOn.getAsBoolean();

        boolean velocityDetecting = isVelocityDetecting.getAsBoolean();

        if (limitOn != lastLimitOn) {

            if (!limitOn) {

                shooter.accelerate_slow();
            } else {

                shooter.accelerate_idle();
            }

            lastLimitOn = limitOn;
        }


        if (limitOn) {
            intake.limitOn();
            led.setNone();
        } else {
            intake.limitOff();
            led.setBlue();


            if (velocityDetecting && shooter.isAsTargetVelocity){

                led.setGreen();  //

            }
        }




    }

}
