// code by jph
package ch.ethz.idsc.owly.demo.glc.delta;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Norm;

public class DeltaGoalManager extends SimpleTrajectoryRegionQuery implements CostFunction {
  final Tensor center;
  final Scalar radius;
  final Scalar maxSpeed;

  public DeltaGoalManager(Tensor center, Tensor radius, Scalar maxSpeed) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, radius)));
    this.center = center;
    this.maxSpeed = maxSpeed;
    if (!radius.Get(0).equals(radius.Get(1)))
      throw new RuntimeException(); // x-y radius have to be equal
    this.radius = radius.Get(0);
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    // return RealScalar.of(trajectory.size());
    Scalar sum = RealScalar.of(0);
    for (int i = 0; i < flow.getU().length(); i++) {
      // Costfunction: integrate (u² +0.1, t)
      // TODO: check theory
      sum = sum.add(Norm._2.of(flow.getU()).add(RealScalar.of(0.1)));
    }
    return sum;
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    Tensor cur_xy = x.extract(0, 2);
    // Heuristic needs to be underestimating: (Euclideandistance-radius) / (MaxControl+Max(|Vectorfield|)
    Scalar dxy = Norm._2.of(cur_xy.subtract(center)).subtract(radius).divide(RealScalar.ONE);
    // Scalar dxy = Norm._2.of(cur_xy.subtract(center)).subtract(radius).divide(maxSpeed);
    return Max.of(dxy, ZeroScalar.get());
    // return ZeroScalar.get();
  }
}
