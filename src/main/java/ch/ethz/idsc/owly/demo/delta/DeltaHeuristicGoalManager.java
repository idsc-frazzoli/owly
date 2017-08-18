// code by jl
package ch.ethz.idsc.owly.demo.delta;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

public class DeltaHeuristicGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final Tensor center;
  private final Scalar radius;
  private final Scalar maxSpeed;
  private final Scalar timeCostScalingFactor;

  // Constructor with Default value in CostScaling
  public DeltaHeuristicGoalManager(Tensor center, Tensor radius, Scalar maxSpeed) {
    this(center, radius, maxSpeed, RealScalar.ONE);
  }

  public DeltaHeuristicGoalManager(Tensor center, Tensor radius, Scalar maxSpeed, Scalar timeCostScalingFactor) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, radius)));
    this.center = center;
    this.maxSpeed = maxSpeed;
    if (!radius.Get(0).equals(radius.Get(1)))
      throw new RuntimeException(); // x-y radius have to be equal
    this.radius = radius.Get(0);
    this.timeCostScalingFactor = timeCostScalingFactor;
  }

  // --
  public DeltaHeuristicGoalManager(Region region, Tensor center, Tensor radius, Scalar maxSpeed) {
    this(region, center, radius, maxSpeed, RealScalar.ONE);
  }

  public DeltaHeuristicGoalManager(Region region, Tensor center, Tensor radius, Scalar maxSpeed, Scalar timeCostScalingFactor) {
    super(new TimeInvariantRegion(region));
    this.center = center;
    this.maxSpeed = maxSpeed;
    if (!radius.Get(0).equals(radius.Get(1)))
      throw new RuntimeException(); // x-y radius have to be equal
    this.radius = radius.Get(0);
    this.timeCostScalingFactor = timeCostScalingFactor;
  }
  // --

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    Scalar sum = Norm._2.of(flow.getU()).add(timeCostScalingFactor);
    // Costfunction: integrate (u^2 +1, t)
    return sum.multiply(StateTimeTrajectories.timeIncrement(from, trajectory));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    // B. Paden: A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning
    // p. 79 Eq: 6.4.14
    // Heuristic needs to be underestimating: (Euclideandistance-radius) / (MaxControl+Max(|Vectorfield|)
    // return RealScalar.ZERO;
    return Ramp.of(Norm._2.of(x.subtract(center)).subtract(radius).divide(maxSpeed));
  }
}
