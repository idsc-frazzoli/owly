// code by jl
package ch.ethz.idsc.owly.demo.delta;

import java.util.List;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class DeltaTrajectoryGoalManager extends DeltaGoalManagerExt {
  // Constructor with Default value in CostScaling
  public DeltaTrajectoryGoalManager(List<Region> goalRegions, Tensor center, Tensor radius, Scalar maxSpeed, Scalar costScalingFactor) {
    super(RegionUnion.wrap(goalRegions), center, radius, maxSpeed, costScalingFactor);
  }

  public DeltaTrajectoryGoalManager(List<Region> goalRegions, Tensor center, Tensor radius, Scalar maxSpeed) {
    this(goalRegions, center, radius, maxSpeed, RealScalar.ONE);
  }

  public DeltaTrajectoryGoalManager(List<Region> goalRegions, Tensor center, Scalar maxSpeed) {
    this(goalRegions, center, Tensors.vector(0, 0), maxSpeed, RealScalar.ONE);
  }
}
