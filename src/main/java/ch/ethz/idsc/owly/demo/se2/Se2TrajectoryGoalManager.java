// code by jl
package ch.ethz.idsc.owly.demo.se2;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.data.Lists;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** objective is minimum time */
public class Se2TrajectoryGoalManager extends TrajectoryGoalManager {
  private final List<StateTime> heuristicTrajectory;
  private final Scalar radius;
  private final Scalar maxSpeed;

  public Se2TrajectoryGoalManager(List<Region> goalRegions, List<StateTime> heuristicTrajectory, Tensor radius, Collection<Flow> controls) {
    super(goalRegions);
    this.heuristicTrajectory = heuristicTrajectory;
    if (!radius.Get(0).equals(radius.Get(1)))
      throw TensorRuntimeException.of(radius); // x-y radius have to be equal
    this.radius = radius.Get(0);
    this.maxSpeed = Se2Controls.maxSpeed(controls);
    System.out.println("HERE maxSpeed " + maxSpeed);
  }

  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode, trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    Tensor center = Lists.getLast(heuristicTrajectory).state().extract(0, 2);
    Tensor cur_xy = x.extract(0, 2);
    return Ramp.of(Norm._2.ofVector(cur_xy.subtract(center)).subtract(radius).divide(maxSpeed));
    // return RealScalar.ZERO;
  }
}
