// code by jph
package ch.ethz.idsc.owly.demo.glc.psu;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.Trajectories;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.ZeroScalar;

class PsuGoalManager extends SimpleTrajectoryRegionQuery implements CostFunction {
  public PsuGoalManager(Tensor radius) {
    super(new TimeInvariantRegion( //
        RegionUnion.of( //
            new EllipsoidRegion(Tensors.vector(+Math.PI, 0), radius), //
            new EllipsoidRegion(Tensors.vector(-Math.PI, 0), radius) //
        )));
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Trajectories.timeIncrement(from, trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return ZeroScalar.get();
  }
}
