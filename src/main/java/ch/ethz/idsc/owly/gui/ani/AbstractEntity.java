// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.state.EpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.ArgMin;

public abstract class AbstractEntity implements RenderInterface, AnimationInterface {
  protected final EpisodeIntegrator episodeIntegrator;

  public AbstractEntity(EpisodeIntegrator episodeIntegrator) {
    this.episodeIntegrator = episodeIntegrator;
  }

  protected List<TrajectorySample> trajectory = null;

  synchronized void setTrajectory(List<TrajectorySample> trajectory) {
    this.trajectory = trajectory;
  }

  @Override
  public final synchronized void integrate(Scalar now) {
    // implementation does not require that current position is perfectly located on trajectory
    Tensor u = fallbackControl(); // default control
    if (Objects.nonNull(trajectory)) {
      int index = indexOfClosestTrajectorySample();
      GlobalAssert.that(index != ArgMin.NOINDEX);
      ++index; // next node has flow control
      if (index < trajectory.size()) {
        GlobalAssert.that(trajectory.get(index).getFlow().isPresent());
        u = trajectory.get(index).getFlow().get().getU();
      } else {
        System.out.println("trajectory finished");
        trajectory = null;
      }
    }
    episodeIntegrator.move(u, now);
  }

  /** @param delay
   * @return estimated location of agent after given delay */
  final synchronized List<TrajectorySample> getFutureTrajectoryUntil(Scalar delay) {
    if (Objects.isNull(trajectory))
      return Collections.singletonList(TrajectorySample.head(episodeIntegrator.tail()));
    int index = indexOfClosestTrajectorySample();
    // TODO JAN this code is almost generic => extract to util class
    Scalar threshold = trajectory.get(index).stateTime().time().add(delay);
    return trajectory.stream().skip(index) //
        .filter(trajectorySample -> Scalars.lessEquals(trajectorySample.stateTime().time(), threshold)) //
        .collect(Collectors.toList());
  }

  /** @param delay
   * @return estimated location of agent after given delay */
  final Tensor getEstimatedLocationAt(Scalar delay) {
    if (Objects.isNull(trajectory))
      return episodeIntegrator.tail().state();
    List<TrajectorySample> relevant = getFutureTrajectoryUntil(delay);
    return relevant.get(relevant.size() - 1).stateTime().state();
  }

  abstract int indexOfClosestTrajectorySample();

  abstract Tensor fallbackControl();

  abstract Scalar delayHint();

  abstract TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal);
}