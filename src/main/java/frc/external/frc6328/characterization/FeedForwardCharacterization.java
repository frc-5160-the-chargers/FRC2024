// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
package frc.external.frc6328.characterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Credits: 6328 repository: <a name = "6328 repository" href="https://github.com/Mechanical-Advantage/RobotCode2023/">(Repository Here)</a>
 */
public class FeedForwardCharacterization extends Command {
    private static final double startDelaySecs = 2.0;
    private static final double rampRateVoltsPerSec = 0.05;

    private final boolean forwards;
    private final boolean isDrive;

    private final FeedForwardCharacterizationData dataPrimary;
    private final FeedForwardCharacterizationData dataSecondary;
    private final Consumer<Double> voltageConsumerSimple;
    private final BiConsumer<Double, Double> voltageConsumerDrive;
    private final Supplier<Double> velocitySupplierPrimary;
    private final Supplier<Double> velocitySupplierSecondary;

    private final Timer timer = new Timer();

    /** Creates a new FeedForwardCharacterization for a differential drive. */
    public FeedForwardCharacterization(
            boolean forwards,
            FeedForwardCharacterizationData leftData,
            FeedForwardCharacterizationData rightData,
            BiConsumer<Double, Double> voltageConsumer,
            Supplier<Double> leftVelocitySupplier,
            Supplier<Double> rightVelocitySupplier,
            Subsystem... requirements
    ) {
        addRequirements(requirements);
        this.forwards = forwards;
        this.isDrive = true;
        this.dataPrimary = leftData;
        this.dataSecondary = rightData;
        this.voltageConsumerSimple = null;
        this.voltageConsumerDrive = voltageConsumer;
        this.velocitySupplierPrimary = leftVelocitySupplier;
        this.velocitySupplierSecondary = rightVelocitySupplier;
    }

    /** Creates a new FeedForwardCharacterization for a simple subsystem. */
    public FeedForwardCharacterization(
            boolean forwards,
            FeedForwardCharacterizationData data,
            Consumer<Double> voltageConsumer,
            Supplier<Double> velocitySupplier,
            Subsystem... requirements
    ) {
        addRequirements(requirements);
        this.forwards = forwards;
        this.isDrive = false;
        this.dataPrimary = data;
        this.dataSecondary = null;
        this.voltageConsumerSimple = voltageConsumer;
        this.voltageConsumerDrive = null;
        this.velocitySupplierPrimary = velocitySupplier;
        this.velocitySupplierSecondary = null;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() < startDelaySecs) {
            if (isDrive) {
                voltageConsumerDrive.accept(0.0, 0.0);
            } else {
                voltageConsumerSimple.accept(0.0);
            }
        } else {
            double voltage = (timer.get() - startDelaySecs) * rampRateVoltsPerSec * (forwards ? 1 : -1);
            if (isDrive) {
                voltageConsumerDrive.accept(voltage, voltage);
            } else {
                voltageConsumerSimple.accept(voltage);
            }
            dataPrimary.add(velocitySupplierPrimary.get(), voltage);
            if (isDrive) {
                dataSecondary.add(velocitySupplierSecondary.get(), voltage);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (isDrive) {
            voltageConsumerDrive.accept(0.0, 0.0);
        } else {
            voltageConsumerSimple.accept(0.0);
        }
        timer.stop();
        dataPrimary.print();
        if (isDrive) {
            dataSecondary.print();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
