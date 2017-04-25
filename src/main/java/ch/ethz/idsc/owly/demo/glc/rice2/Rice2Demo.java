// code by jph
package ch.ethz.idsc.owly.demo.glc.rice2;

import ch.ethz.idsc.owly.glc.adapter.EllipsoidRegion;
import ch.ethz.idsc.owly.glc.adapter.EmptyRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.adapter.ZeroHeuristic;
import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.owly.math.integrator.MidpointIntegrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** "Mobility and Autonomous Reconfiguration of Marsokhod" */
public class Rice2Demo {
  public static void main(String[] args) {
    Integrator integrator = new MidpointIntegrator();
    Scalar timeStep = RealScalar.of(.25);
    Tensor partitionScale = Tensors.vector(2, 2, 2, 2);
    Controls controls = new Rice2Controls(15);
    int trajectorySize = 5;
    CostFunction costFunction = new MinTimeCost();
    Heuristic heuristic = new ZeroHeuristic();
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new EllipsoidRegion(Tensors.vector(3, 3, -1, 0), Tensors.vector(1, 1, .5, 1)) //
        ));
    TrajectoryRegionQuery obstacleQuery = //
        new EmptyRegionQuery();
    // new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
    // RegionUnion.of( //
    // new EllipsoidRegion(Tensors.vector(+3, +1), Tensors.vector(1.75, .75)), // speed limit along the way
    // new EllipsoidRegion(Tensors.vector(-2, +0), Tensors.vector(1, 1)) // block to the left
    // )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrator, timeStep, partitionScale, controls, trajectorySize, costFunction, heuristic, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0, 0));
    trajectoryPlanner.plan(25);
    // TODO keep trying to improve path to goal for a few iterations...?
    Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
    trajectory.print();
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
