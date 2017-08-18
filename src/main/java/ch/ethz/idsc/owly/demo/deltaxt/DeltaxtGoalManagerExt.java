// code by jl
package ch.ethz.idsc.owly.demo.deltaxt;

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
import ch.ethz.idsc.tensor.sca.Power;
import ch.ethz.idsc.tensor.sca.Ramp;
import ch.ethz.idsc.tensor.sca.Sqrt;

public class DeltaxtGoalManagerExt extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final Tensor center;
  private final Tensor radius;
  private final Scalar maxSpeed;
  private final Scalar timeCostScalingFactor; // TODO JONAS not used

  // Constructor with Default value in CostScaling
  public DeltaxtGoalManagerExt(Tensor center, Tensor radius, Scalar maxSpeed) {
    this(center, radius, maxSpeed, RealScalar.ONE);
  }

  public DeltaxtGoalManagerExt(Tensor center, Tensor radius, Scalar maxSpeed, Scalar timeCostScalingFactor) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, radius)));
    this.center = center;
    this.maxSpeed = maxSpeed;
    if (!radius.Get(0).equals(radius.Get(1)))
      throw new RuntimeException(); // x-y radius have to be equal
    this.radius = radius;
    this.timeCostScalingFactor = timeCostScalingFactor;
  }

  // --
  public DeltaxtGoalManagerExt(Region region, Tensor center, Tensor radius, Scalar maxSpeed) {
    this(region, center, radius, maxSpeed, RealScalar.ONE);
  }

  public DeltaxtGoalManagerExt(Region region, Tensor center, Tensor radius, Scalar maxSpeed, Scalar timeCostScalingFactor) {
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
    // Costfunction: t
    return StateTimeTrajectories.timeIncrement(from, trajectory);
  }

  /** Ellipsoid with axis: a,b and vector from Center: v = (x,y)
   * specific radius at intersection:
   * r :
   * 
   * a*b * ||v||
   * ---------------
   * sqrt(a²y² + b²x²) */
  @Override
  public Scalar minCostToGoal(Tensor x) {
    // B. Paden: A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning
    // p. 79 Eq: 6.4.14
    // Heuristic needs to be underestimating: (Euclideandistance-radius) / (MaxControl+Max(|Vectorfield|)
    int toIndex = x.length() - 1;
    Tensor r2x = x.extract(0, toIndex);
    Tensor r2Center = center.extract(0, toIndex);
    Tensor r2Vector = r2x.subtract(r2Center);
    Tensor r2Radius = radius.extract(0, toIndex);
    Scalar root = Sqrt.of(Power.of(r2Radius.Get(0).multiply(r2Vector.Get(1)), 2)//
        .add(Power.of(r2Radius.Get(1).multiply(r2Vector.Get(0)), 2)));
    Scalar specificRadius = radius.Get(0).multiply(radius.Get(1)).multiply(Norm._2.of(r2x.subtract(r2Center))).divide(root);
    return Ramp.of(Norm._2.of(r2x.subtract(r2Center)).subtract(specificRadius).divide(maxSpeed)); // <- do not change
  }

  @Override
  public boolean hasHeuristic() {
    return true;
  }
}
