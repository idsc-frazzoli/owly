// code by jl
package ch.ethz.idsc.owly.demo.delta;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

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
}
