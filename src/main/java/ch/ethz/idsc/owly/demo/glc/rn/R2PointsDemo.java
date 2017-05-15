// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.stream.IntStream;

import ch.ethz.idsc.owly.glc.adapter.RnPointcloudRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class R2PointsDemo {
  static Tensor createRandom(int num, Tensor width) {
    Random random = new Random();
    Tensor points = Tensors.empty();
    IntStream.range(0, num).boxed() //
        .forEach(i -> points.append(width.pmul(Tensors.vector(random.nextDouble(), random.nextDouble()))));
    return points;
  }

  public static void main(String[] args) {
    Tensor partitionScale = Tensors.vector(5, 5);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(new EulerIntegrator(), RationalScalar.of(1, 8), 4);
    Collection<Flow> controls = R2Controls.createControls(20);
    RnGoalManager rnGoal = new RnGoalManager(Tensors.vector(5, 5), DoubleScalar.of(.2));
    Tensor points = createRandom(10, Tensors.vector(4, 4));
    // Tensors.matrix(new Number[][] { //
    // { 2.5, 1 }, { 2.5, 0 }, { 1.5, -1.5 }, { 0, 2 }, { 3.5, -0.5 } //
    // });
    TrajectoryRegionQuery obstacleQuery = // new EmptyRegionQuery();
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RnPointcloudRegion.create(points, RealScalar.of(0.6))));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, rnGoal, rnGoal, obstacleQuery);
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    int iters = Expand.maxSteps(trajectoryPlanner, 1000);
    System.out.println(iters);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    Gui.glc(trajectoryPlanner);
  }
}
