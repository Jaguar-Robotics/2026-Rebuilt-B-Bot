// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * How much fuel the hopper holds. Plain counting logic (no physics), so it can be unit tested. The simulated moving mass
 * is interpolated between the empty and fully loaded masses by how full the hopper is.
 */
public class FuelHopper {
    private int capacity;
    private final double emptyMassKg;
    private final double fullMassKg;
    private int count = 0;

    public FuelHopper(int capacity, double emptyMassKg, double fullMassKg) {
        this.capacity = capacity;
        this.emptyMassKg = emptyMassKg;
        this.fullMassKg = fullMassKg;
    }

    public int getCount() {
        return count;
    }

    /** Change how much the hopper holds (live tuning). A hopper already over the new capacity just counts as full. */
    public void setCapacity(int capacity) {
        this.capacity = Math.max(1, capacity);
    }

    public int getCapacity() {
        return capacity;
    }

    public boolean isFull() {
        return count >= capacity;
    }

    public boolean isEmpty() {
        return count <= 0;
    }

    /** @return true if a fuel was taken in, false if the hopper is full. */
    public boolean intake() {
        if (isFull()) return false;
        count++;
        return true;
    }

    /** @return true if a fuel was released, false if the hopper is empty. */
    public boolean outtake() {
        if (isEmpty()) return false;
        count--;
        return true;
    }

    public void clear() {
        count = 0;
    }

    /** Moving mass of the mechanism with the current fuel on board. */
    public double getMassKg() {
        return emptyMassKg + (fullMassKg - emptyMassKg) * count / capacity;
    }
}
