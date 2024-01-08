package frc.robot.hardware.subsystems.odometry.threads;
// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.ConstantsKt;
import frc.robot.LogInputsKt;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 * <p>
 * Note: This was originally Mechanical Advantage's SparkMaxOdometryThread; however,
 * this is the only thread we can use because measurements must be synchronous,
 * and we aren't using falcons for swerve.
 */
@SuppressWarnings("ALL")
public class OdometryThread {
    private final List<DoubleSupplier> signals = new ArrayList<>();
    private final List<Queue<Double>> queues = new ArrayList<>();

    private final Notifier notifier;
    private static OdometryThread instance = null;

    public static OdometryThread getInstance() {
        if (instance == null) {
            instance = new OdometryThread();
        }
        return instance;
    }

    private OdometryThread() {
        notifier = new Notifier(this::periodic);
        notifier.setName("SparkMaxOdometryThread");
        notifier.startPeriodic(1.0 / ConstantsKt.ODOMETRY_UPDATE_FREQUENCY_HZ );
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        LogInputsKt.OdometryLog.getUpdateInputsLock().lock();
        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            LogInputsKt.OdometryLog.getUpdateInputsLock().unlock();
        }
        return queue;
    }

    private void periodic() {
        LogInputsKt.OdometryLog.getUpdateInputsLock().lock();
        try {
            for (int i = 0; i < signals.size(); i++) {
                queues.get(i).offer(signals.get(i).getAsDouble());
            }
        } finally {
            LogInputsKt.OdometryLog.getUpdateInputsLock().unlock();
        }
    }
}
