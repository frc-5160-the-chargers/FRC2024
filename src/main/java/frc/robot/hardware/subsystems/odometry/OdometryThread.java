package frc.robot.hardware.subsystems.odometry;
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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.constants.DrivetrainConstantsKt;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.ArrayDeque;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 * <p>
 * Note: This was originally Mechanical Advantage's SparkMaxOdometryThread;
 * however, we renamed it because this is the only thread we can use, as measurements must be synchronous,
 * and we aren't using falcons for swerve.
 */
@SuppressWarnings("ALL")
public class OdometryThread {
    /**
     * The reentrant lock used for odometry.
     */
    public static final ReentrantLock ODOMETRY_LOCK = new ReentrantLock();

    private List<DoubleSupplier> signals = new ArrayList<>();
    private List<Supplier<Pose2d>> poseSignals = new ArrayList<>();


    private List<Queue<Double>> queues = new ArrayList<>();
    private List<Queue<Double>> timestampQueues = new ArrayList<>();


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
        notifier.setName("OdometryThread");
    }

    public void start() {
        if (timestampQueues.size() > 0) {
            notifier.startPeriodic(1.0 / DrivetrainConstantsKt.ODOMETRY_UPDATE_FREQUENCY_HZ);
        }
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayDeque<>(100);
        ODOMETRY_LOCK.lock();
        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayDeque<>(100);
        ODOMETRY_LOCK.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    private void periodic() {
        ODOMETRY_LOCK.lock();
        double timestamp = Logger.getRealTimestamp() / 1e6;
        try {
            for (int i = 0; i < signals.size(); i++) {
                queues.get(i).offer(signals.get(i).getAsDouble());
            }
            for (int i = 0; i < timestampQueues.size(); i++) {
                timestampQueues.get(i).offer(timestamp);
            }
        } finally {
            ODOMETRY_LOCK.unlock();
        }
    }
}
