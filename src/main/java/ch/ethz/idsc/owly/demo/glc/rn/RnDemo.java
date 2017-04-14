// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.adapter.ZeroHeuristic;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.DynamicalSystem;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.util.rn.RnSphericalRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class RnDemo {
  public static void main(String[] args) {
    DynamicalSystem dynamicalSystem = new RnSingleIntegrator(1);
    Tensor controls = RnControls.createR2RadialControls(6);
    // System.out.println(Pretty.of(controls));
    CostFunction costFunction = new MinTimeCost();
    Heuristic heuristic = new ZeroHeuristic();
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new RnSphericalRegion(Tensors.vector(10, 0), RealScalar.of(1))));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new RnSphericalRegion(Tensors.vector(5, 0), RealScalar.of(3))));
    // ---
    TrajectoryPlanner trajectoryPlanner = new TrajectoryPlanner( //
        dynamicalSystem, controls, costFunction, heuristic, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    trajectoryPlanner.plan();
    Trajectory trajectory = trajectoryPlanner.getPathFromGoalToRoot();
    trajectory.print();
  }
}
