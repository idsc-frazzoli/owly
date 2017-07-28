// code by jl
package ch.ethz.idsc.owly.demo.delta;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.Trajectories;
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
    super(RegionUnion.wrap(goalRegions), Trajectories.getLast(heuristicTrajectory).x(), radius, maxSpeed, costScalingFactor);
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
    // B. Paden: A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning
    // p. 79 Eq: 6.4.14
    // Heuristic needs to be underestimating: (Euclideandistance-radius) / (MaxControl+Max(|Vectorfield|)
    Scalar bestDistance = DoubleScalar.POSITIVE_INFINITY;
    StateTime closestState = null;
    if (x.length() != heuristicTrajectory.get(0).x().length()) {
      throw new RuntimeException();
    }
    for (StateTime entry : heuristicTrajectory) {
      Scalar distance = Norm._2.of(entry.x().subtract(x));
      if (Scalars.lessThan(distance, bestDistance)) {
        bestDistance = distance;
        closestState = entry;
      }
    }
    Scalar trajectoryIndex = RealScalar.of(heuristicTrajectory.size() - heuristicTrajectory.indexOf(closestState));
    Scalar magicConstant = maxSpeed.multiply(RealScalar.of(0.8));
    return Ramp.of(bestDistance.subtract(radius).divide(maxSpeed).add(trajectoryIndex.multiply(magicConstant)));
  }
}
