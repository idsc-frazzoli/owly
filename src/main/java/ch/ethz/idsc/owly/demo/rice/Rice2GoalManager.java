// code by jph
package ch.ethz.idsc.owly.demo.rice;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

public class Rice2GoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final Tensor center;
  private final Scalar radius;

  public Rice2GoalManager(Tensor center, Tensor radius) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, radius)));
    this.center = center;
    if (!radius.Get(0).equals(radius.Get(1)))
      throw new RuntimeException(); // x-y radius have to be equal
    this.radius = radius.Get(0);
  }

  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode, trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    Tensor pc = x.extract(0, 2);
    Tensor pd = center.extract(0, 2);
    Scalar mindist = Ramp.of(Norm._2.ofVector(pc.subtract(pd)).subtract(radius));
    return mindist; // .divide(1 [m/s]), since max velocity == 1 => division is obsolete
  }
}
