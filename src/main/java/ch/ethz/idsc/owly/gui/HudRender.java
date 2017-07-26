// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;

class HudRender implements RenderInterface {
  private static final Color SHADING = new Color(0, 0, 0, 64);
  // ---
  @SuppressWarnings("unused")
  private final TrajectoryPlanner trajectoryPlanner;

  HudRender(TrajectoryPlanner trajectoryPlanner) {
    this.trajectoryPlanner = trajectoryPlanner;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    graphics.setColor(SHADING);
    // for now, do nothing
  }
}
