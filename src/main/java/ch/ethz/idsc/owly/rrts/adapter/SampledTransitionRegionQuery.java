// code by jph
package ch.ethz.idsc.owly.rrts.adapter;

import java.util.Collection;
import java.util.Collections;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.rrts.core.Transition;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;

public class SampledTransitionRegionQuery implements TransitionRegionQuery {
  private final TrajectoryRegionQuery trajectoryRegionQuery;
  private final Scalar dt;

  public SampledTransitionRegionQuery(TrajectoryRegionQuery trajectoryRegionQuery, Scalar dt) {
    this.trajectoryRegionQuery = trajectoryRegionQuery;
    this.dt = dt;
  }

  @Override
  public boolean isDisjoint(Transition transition) {
    List<StateTime> list = transition.sampled(RealScalar.ZERO, RealScalar.ZERO, dt);
    return trajectoryRegionQuery.isDisjoint(list);
  }

  public Collection<StateTime> getSparseDiscoveredMembers() {
    if (trajectoryRegionQuery instanceof SimpleTrajectoryRegionQuery) {
      SimpleTrajectoryRegionQuery simpleTrajectoryRegionQuery = (SimpleTrajectoryRegionQuery) trajectoryRegionQuery;
      return simpleTrajectoryRegionQuery.getSparseDiscoveredMembers();
    }
    return Collections.emptyList();
  }
}
