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
import ch.ethz.idsc.tensor.Tensor;

// TODO first API draft, unify with se2entity and abstract entity
public abstract class PolicyEntity implements AnimationInterface, RenderInterface {
  protected final EpisodeIntegrator episodeIntegrator;
  public TrajectoryRegionQuery obstacleQuery = null;

  public PolicyEntity(EpisodeIntegrator episodeIntegrator) {
    this.episodeIntegrator = episodeIntegrator;
  }

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
