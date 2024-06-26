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

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.GlobalConstantsKt;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 * <p>
 * Note: This was originally Mechanical Advantage's SparkMaxOdometryThread;
 * however, we renamed it because this is the only thread we can use, as measurements must be synchronous,
 * and the PhoenixOdometryThread
 */
@SuppressWarnings("ALL")
public class OdometryThread {
    /**
     * The reentrant lock used for odometry.
     */
    public static final ReentrantLock ODOMETRY_LOCK = new ReentrantLock();

    private List<Supplier<OptionalDouble>> signals = new ArrayList<>();
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
        notifier.setName("SparkMaxOdometryThread");
    }

    public void start() {
        if (timestampQueues.size() > 0) {
            notifier.startPeriodic(1.0 / GlobalConstantsKt.ODOMETRY_UPDATE_FREQUENCY_HZ);
        }
    }

    public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
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
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
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
            double[] values = new double[signals.size()];
            boolean isValid = true;
            for (int i = 0; i < signals.size(); i++) {
                OptionalDouble value = signals.get(i).get();
                if (value.isPresent()) {
                    values[i] = value.getAsDouble();
                } else {
                    isValid = false;
                    break;
                }
            }
            if (isValid) {
                for (int i = 0; i < queues.size(); i++) {
                    queues.get(i).offer(values[i]);
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
            }
        } finally {
            ODOMETRY_LOCK.unlock();
        }
    }
}
