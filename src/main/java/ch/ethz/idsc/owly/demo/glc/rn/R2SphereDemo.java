// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.EllipsoidRegion;
import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
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
import ch.ethz.idsc.owly.math.integrator.EulerIntegrator;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class R2SphereDemo {
  public static void main(String[] args) {
    Integrator integrator = new EulerIntegrator();
    final Scalar timeStep = RationalScalar.of(1, 8);
    Tensor partitionScale = Tensors.vector(4, 4);
    Controls controls = new R2Controls(20);
    int trajectorySize = 10;
    CostFunction costFunction = new MinTimeCost();
    RnGoal rnGoal = new RnGoal(Tensors.vector(5, 0), DoubleScalar.of(.2));
    TrajectoryRegionQuery obstacleQuery = // new EmptyRegionQuery();
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(4, 3), Tensors.vector(2, 2)), //
                new EllipsoidRegion(Tensors.vector(2.5, 0), Tensors.vector(2, 2)) //
            )));
    Heuristic heuristic = rnGoal;
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(rnGoal));
    // TrajectoryRegionQuery obstacleQuery = obstacleQuery;
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrator, timeStep, partitionScale, controls, trajectorySize, costFunction, heuristic, goalQuery, obstacleQuery);
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    int iters = trajectoryPlanner.plan(1000);
    System.out.println(iters);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectory.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
