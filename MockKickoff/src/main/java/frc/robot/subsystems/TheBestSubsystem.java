package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Catapult;

public class TheBestSubsystem extends SubsystemBase {
    Catapult catapult;

    public TheBestSubsystem() {
        catapult = new Catapult();
    }

    public CommandBase shootYogaBall() {
        return runOnce(() -> {catapult.shoot(4);
        });
    }

    public CommandBase catchYogaBall() {
        return runOnce(() -> {catapult.shoot(0);
        });
    }

    public CommandBase handleYogaBall(double rps) {
        return runOnce(() -> {catapult.shoot(rps);
        });
    }


}