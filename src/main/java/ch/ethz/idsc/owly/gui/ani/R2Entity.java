// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.OwlyLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.state.EpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.ArgMin;
import ch.ethz.idsc.tensor.red.Norm;

public class R2Entity implements AnimationInterface, RenderInterface {
  /** preserve 1[s] of the former trajectory */
  public static final Scalar DELAY_HINT = RealScalar.ONE;
  // ---
  StateSpaceModel ssm = SingleIntegratorStateSpaceModel.INSTANCE;
  EpisodeIntegrator episodeIntegrator = new SimpleEpisodeIntegrator( //
      ssm, //
      EulerIntegrator.INSTANCE, //
      new StateTime(Tensors.vector(0, 0), RealScalar.ZERO));
  // ---
  private List<TrajectorySample> trajectory = null;

  synchronized void setTrajectory(List<TrajectorySample> trajectory) {
    this.trajectory = trajectory;
  }

  // /** @return possibly null */
  // List<TrajectorySample> getTrajectory() {
  // return trajectory;
  // }
  /** @return index of sample of trajectory that is closest to current position */
  private synchronized int indexOfClosestTrajectorySample() {
    final Tensor x = episodeIntegrator.tail().state();
    return ArgMin.of(Tensor.of(trajectory.stream() //
        .map(TrajectorySample::stateTime) //
        .map(StateTime::state) //
        .map(state -> Norm._2SQUARED.of(state.subtract(x)))));
  }

  @Override
  public synchronized void integrate(Scalar now) {
    // implementation does not require that current position is perfectly located on trajectory
    Tensor u = Tensors.vector(0, 0); // default control
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

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // indicate current position
      Tensor state = episodeIntegrator.tail().state();
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(128 - 64, 128, 128 - 64, 128 + 64));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
    { // indicate position 1[s] into the future
      Tensor state = getEstimatedLocationAt(DELAY_HINT);
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 128 - 64, 128 + 64));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
  }

  /** @param delay
   * @return estimated location of agent after given delay */
  Tensor getEstimatedLocationAt(Scalar delay) {
    if (Objects.isNull(trajectory))
      return episodeIntegrator.tail().state();
    List<TrajectorySample> relevant = getFutureTrajectoryUntil(delay);
    return relevant.get(relevant.size() - 1).stateTime().state();
  }

  /** @param delay
   * @return estimated location of agent after given delay */
  synchronized List<TrajectorySample> getFutureTrajectoryUntil(Scalar delay) {
    if (Objects.isNull(trajectory))
      return Collections.singletonList(TrajectorySample.head(episodeIntegrator.tail()));
    int index = indexOfClosestTrajectorySample();
    // TODO JAN this code is almost generic => extract to util class
    Scalar threshold = trajectory.get(index).stateTime().time().add(delay);
    return trajectory.stream().skip(index) //
        .filter(trajectorySample -> Scalars.lessEquals(trajectorySample.stateTime().time(), threshold)) //
        .collect(Collectors.toList());
  }
}
