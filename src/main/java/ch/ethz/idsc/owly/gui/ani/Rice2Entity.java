// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

import ch.ethz.idsc.owly.demo.rice.Rice2StateSpaceModel;
import ch.ethz.idsc.owly.gui.OwlyLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.state.EpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;

@Deprecated
public class Rice2Entity implements AnimationInterface, RenderInterface {
  StateSpaceModel ssm = new Rice2StateSpaceModel(RealScalar.of(1.5));
  EpisodeIntegrator episodeIntegrator = new SimpleEpisodeIntegrator( //
      ssm, //
      EulerIntegrator.INSTANCE, //
      new StateTime(Tensors.vector(0, 0), RealScalar.ZERO));

  @Override
  public void integrate(Scalar now) {
    episodeIntegrator.move(Tensors.vector(.3), now);
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    StateTime stateTime = episodeIntegrator.tail();
    Point2D p = owlyLayer.toPoint2D(stateTime.state());
    graphics.setColor(Color.BLACK);
    graphics.draw(new Rectangle2D.Double(p.getX(), p.getY(), 2, 2));
  }
}
