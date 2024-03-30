// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.A_Star;

import edu.wpi.first.math.geometry.Translation2d;

public class Node
{
    public double x, y;
    public boolean obstacle;
    public Node parent = null;

    private double g_cost, h_cost, f_cost;

    public Node(double _x, double _y, boolean _obstacle) {
        x = _x;
        y = _y;
        obstacle = _obstacle;
    }

    public Node(Translation2d translation, boolean _obstacle) {
        x = translation.getX();
        y = translation.getY();
        obstacle = _obstacle;
    }

    public void set_g_cost(double _g_cost) {
        g_cost = _g_cost;
    }
    public void set_h_cost(double _h_cost) {
        h_cost = _h_cost;
    }
    public double get_f_cost() {
        return h_cost + g_cost;
    }
    public double get_g_cost() {
        return g_cost;
    }
    public double get_h_cost() {
        return h_cost;
    }

    public void set_parent(Node _parent) {
        parent = _parent;
    }
    public Double[] getLocation() {
        Double[] location = {x, y};
        return location;
    }
    private double round(double x) {
        return Math.round(1000.0*x)/1000.0;
    }

    public Node toNearestGrid(Double[] objectSize) {
        x -= x%objectSize[0];
        y -= y%objectSize[1];
        x = round(x);
        y = round(y);
        return this;
    }
}
