// code by jl
package ch.ethz.idsc.owly.demo.delta;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

public class DeltaTrajectoryGoalManager extends DeltaGoalManagerExt implements TrajectoryGoalManager {
  private final List<StateTime> heuristicTrajectory;

  public DeltaTrajectoryGoalManager(List<Region> goalRegions, List<StateTime> heuristicTrajectory, Tensor radius, Scalar maxSpeed, Scalar costScalingFactor) {
    super(RegionUnion.wrap(goalRegions), StateTimeTrajectories.getLast(heuristicTrajectory).state(), radius, maxSpeed, costScalingFactor);
    this.heuristicTrajectory = heuristicTrajectory;
  }

  // Constructor with Default value in CostScaling
  public DeltaTrajectoryGoalManager(List<Region> goalRegions, List<StateTime> heuristicTrajectory, Tensor radius, Scalar maxSpeed) {
    this(goalRegions, heuristicTrajectory, radius, maxSpeed, RealScalar.ONE);
  }

  public DeltaTrajectoryGoalManager(List<Region> goalRegions, List<StateTime> heuristicTrajectory, Scalar maxSpeed) {
    this(goalRegions, heuristicTrajectory, Tensors.vector(0, 0), maxSpeed, RealScalar.ONE);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    // Heuristic needs to be underestimating: (Euclideandistance-radius) / (MaxControl+Max(|Vectorfield|)
    Scalar bestDistance = DoubleScalar.POSITIVE_INFINITY;
    StateTime closestState = null;
    if (x.length() != heuristicTrajectory.get(0).state().length()) {
      throw new RuntimeException();
    }
    for (StateTime entry : heuristicTrajectory) {
      Scalar distance = Norm._2.of(entry.state().subtract(x));
      if (Scalars.lessThan(distance, bestDistance)) {
        bestDistance = distance;
        closestState = entry;
      }
    }
    Scalar trajectoryIndex = RealScalar.of(heuristicTrajectory.size() - heuristicTrajectory.indexOf(closestState));
    Scalar trajectoryToGoalDistance = maxSpeed.multiply(RealScalar.of(5)).divide(RealScalar.of(12)).multiply(trajectoryIndex);
    // TODO JONAS:Heuristic needs to be consistent with brians rules
    return Ramp.of(trajectoryToGoalDistance);
    // return Ramp.of(bestDistance.subtract(radius).divide(maxSpeed).add(trajectoryToGoalDistance));
  }
}
