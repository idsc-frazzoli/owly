// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.Objects;

import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.se2.Se2Utils;
import ch.ethz.idsc.owly.math.state.EpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.subare.core.DiscreteModel;
import ch.ethz.idsc.subare.core.Policy;
import ch.ethz.idsc.subare.core.util.PolicyWrap;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

// TODO first API draft, unify with se2entity and abstract entity
public abstract class PolicyEntity implements AnimationInterface, DiscreteModel, RenderInterface {
  private final EpisodeIntegrator episodeIntegrator;
  public TrajectoryRegionQuery obstacleQuery = null;
  protected Policy policy;

  public PolicyEntity(EpisodeIntegrator episodeIntegrator) {
    this.episodeIntegrator = episodeIntegrator;
  }

  @Override // from AnimationInterface
  public final void integrate(Scalar now) {
    // implementation does not require that current position is perfectly located on trajectory
    // Tensor u = fallbackControl(); // default control
    StateTime stateTime = getStateTimeNow();
    Tensor state = represent(stateTime); // may be augmented state time, and/or observation etc.
    // System.out.println(state);
    PolicyWrap policyWrap = new PolicyWrap(policy);
    Tensor actions = actions(state);
    Tensor u = policyWrap.next(state, actions);
    episodeIntegrator.move(u, now);
  }

  /** @return control vector to feed the episodeIntegrator in case no planned trajectory is available */
  protected abstract Tensor fallbackControl();

  public abstract Tensor represent(StateTime stateTime);

  public final StateTime getStateTimeNow() {
    return episodeIntegrator.tail();
  }

  protected abstract Tensor shape();

  private boolean obstacleQuery_isDisjoint(StateTime stateTime) {
    return Objects.isNull(obstacleQuery) || !obstacleQuery.isMember(stateTime);
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    { // indicate current position
      final StateTime stateTime = getStateTimeNow();
      Color color = obstacleQuery_isDisjoint(stateTime) //
          ? new Color(64, 64, 64, 128)
          : new Color(255, 64, 64, 128);
      geometricLayer.pushMatrix(Se2Utils.toSE2Matrix(stateTime.state()));
      graphics.setColor(color);
      graphics.fill(geometricLayer.toPath2D(shape()));
      geometricLayer.popMatrix();
    }
    { // draw mouse
      Color color = new Color(0, 128, 255, 192);
      StateTime stateTime = new StateTime(geometricLayer.getMouseSe2State(), getStateTimeNow().time());
      if (!obstacleQuery_isDisjoint(stateTime))
        color = new Color(255, 96, 96, 128);
      geometricLayer.pushMatrix(geometricLayer.getMouseSe2Matrix());
      graphics.setColor(color);
      graphics.fill(geometricLayer.toPath2D(shape()));
      geometricLayer.popMatrix();
    }
  }
}
