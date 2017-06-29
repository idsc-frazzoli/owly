// code by jl
package ch.ethz.idsc.owly.demo.delta;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

//TODO extend from DeltaGoalManager extended, so that costIncrement and minCostToGoal are not double
public class DeltaTrajectoryGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final List<StateTime> goalTrajectory;
  private final Scalar radius;
  private final Scalar maxSpeed;
  private final Scalar costScalingFactor;

  // Constructor with Default value in CostScaling
  public DeltaTrajectoryGoalManager(List<StateTime> goalTrajectory, Tensor radius, Scalar maxSpeed) {
    this(goalTrajectory, radius, maxSpeed, RealScalar.ONE);
  }

  public DeltaTrajectoryGoalManager(List<StateTime> goalTrajectory, Tensor radius, Scalar maxSpeed, Scalar costScalingFactor) {
    // only for comliling reasons
    super(new TimeInvariantRegion(new EllipsoidRegion(Tensors.vector(0.5, 1.0), radius)));
    StateTimeRegion trajectoryRegion = null;
    // TODO how to work around super needs to be first
    // super(trajectoryRegion);
    List<Region> regionList = new ArrayList<>();
    Iterator<StateTime> iterator = goalTrajectory.iterator();
    while (iterator.hasNext()) {
      regionList.add(new EllipsoidRegion(iterator.next().x(), radius));
    }
    trajectoryRegion = new TimeInvariantRegion(RegionUnion.of(regionList));
    // --
    this.goalTrajectory = goalTrajectory;
    this.maxSpeed = maxSpeed;
    if (!radius.Get(0).equals(radius.Get(1)))
      throw new RuntimeException(); // x-y radius have to be equal
    this.radius = radius.Get(0);
    this.costScalingFactor = costScalingFactor;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    Scalar sum = Norm._2.of(flow.getU()).add(costScalingFactor);
    // Costfunction: integrate (u^2 +1, t)
    return sum.multiply(Trajectories.timeIncrement(from, trajectory));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    // B. Paden: A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning
    // p. 79 Eq: 6.4.14
    // Heuristic needs to be underestimating: (Euclideandistance-radius) / (MaxControl+Max(|Vectorfield|)
    // last State from trajectory is the ultimate goal
    Iterator<StateTime> iterator = goalTrajectory.listIterator(goalTrajectory.size() - 1);
    // TODO check if last state
    Tensor center = iterator.next().x();
    return Ramp.of(Norm._2.of(x.subtract(center)).subtract(radius).divide(maxSpeed));
  }
}
