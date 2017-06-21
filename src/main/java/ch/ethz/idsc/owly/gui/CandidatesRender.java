// code by jl
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;

import ch.ethz.idsc.owly.glc.core.AbstractAnyTrajectoryPlanner;

public class CandidatesRender implements AbstractRender {
  private final AbstractAnyTrajectoryPlanner abstractAnyTrajectoryPlanner;

  public CandidatesRender(AbstractAnyTrajectoryPlanner abstractAnyTrajectoryPlanner) {
    this.abstractAnyTrajectoryPlanner = abstractAnyTrajectoryPlanner;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    graphics.setColor(Color.black);
    graphics.drawString("candidates could appear in view", 50, 50);
    // TODO JONAS abstractAnyTrajectoryPlanner."get candidates set" and visualize them
  }
}
