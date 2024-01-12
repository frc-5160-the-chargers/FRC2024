// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
package frc.external.frc6328.characterization;

import java.util.LinkedList;
import java.util.List;

public class FeedForwardCharacterizationData {
    private final String name;
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    public FeedForwardCharacterizationData(String name) {
        this.name = name;
    }

    public void add(double velocity, double voltage) {
        if (Math.abs(velocity) > 1E-4) {
            velocityData.add(Math.abs(velocity));
            voltageData.add(Math.abs(voltage));
        }
    }

    public void print() {
        if (velocityData.isEmpty() || voltageData.isEmpty()) {
            System.out.println("DATA IS EMPTY");
            return;
        }

        PolynomialRegression regression =
                new PolynomialRegression(
                        velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
                        voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
                        1);

        System.out.println("FF Characterization Results (" + name + "):");
        System.out.println("\tCount=" + velocityData.size());
        System.out.printf("\tR2=%.5f%n", regression.R2());
        System.out.printf("\tkS=%.5f%n", regression.beta(0));
        System.out.printf("\tkV=%.5f%n", regression.beta(1));
    }
}
