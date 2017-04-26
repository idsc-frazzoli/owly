// code by jph
package ch.ethz.idsc.owly.demo.glc.rice2;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.EllipsoidRegion;
import ch.ethz.idsc.owly.glc.adapter.HyperplaneRegion;
import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.adapter.ZeroHeuristic;
import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.RegionUnion;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.owly.math.integrator.MidpointIntegrator;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** "Mobility and Autonomous Reconfiguration of Marsokhod" */
public class Rice2Demo {
  public static void main(String[] args) {
    Integrator integrator = new MidpointIntegrator();
    Scalar timeStep = RationalScalar.of(1, 2);
    Tensor partitionScale = Tensors.vector(4, 4, 4, 4);
    Controls controls = new Rice2Controls(RealScalar.of(.5), 15);
    int trajectorySize = 5;
    CostFunction costFunction = new MinTimeCost();
    Heuristic heuristic = new ZeroHeuristic();
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new EllipsoidRegion(Tensors.vector(3, 3, -1, 0), Tensors.vector(1, 1, .5, .5)) //
        ));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new HyperplaneRegion(Tensors.vector(0, 0, 0, 1), RealScalar.of(-0.1)), //
                // block to the left
                new EllipsoidRegion(Tensors.vector(-2, +0), Tensors.vector(1, 5)) //
            )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrator, timeStep, partitionScale, controls, trajectorySize, costFunction, heuristic, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0, 0));
    int iters = trajectoryPlanner.plan(1000);
    System.out.println(iters);
    // TODO keep trying to improve path to goal for a few iterations...?
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectory.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
