// code by jph
package ch.ethz.idsc.owly.gui.ren;

import java.awt.Color;
import java.awt.Graphics2D;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;

class HudRender implements RenderInterface {
  private static final Color SHADING = new Color(0, 0, 0, 64);
  // ---
  @SuppressWarnings("unused")
  private final TrajectoryPlanner trajectoryPlanner;

  HudRender(TrajectoryPlanner trajectoryPlanner) {
    this.trajectoryPlanner = trajectoryPlanner;
  }

  @Override
  public void render(GeometricLayer owlyLayer, Graphics2D graphics) {
    graphics.setColor(SHADING);
    // TODO decide on purpose of HudRender, for now, do nothing
  }
}
