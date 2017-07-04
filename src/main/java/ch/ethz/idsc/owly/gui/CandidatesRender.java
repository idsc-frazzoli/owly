// code by jl
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.Map;
import java.util.Set;

import ch.ethz.idsc.owly.glc.core.CandidatePair;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.tensor.Tensor;

public class CandidatesRender implements AbstractRender {
  private final OptimalAnyTrajectoryPlanner OptimalAnyTrajectoryPlanner;

  public CandidatesRender(OptimalAnyTrajectoryPlanner OptimalAnyTrajectoryPlanner) {
    this.OptimalAnyTrajectoryPlanner = OptimalAnyTrajectoryPlanner;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    graphics.setColor(Color.black);
    graphics.drawString("candidates could appear in view", 50, 50);
    // TODO JONAS abstractAnyTrajectoryPlanner."get candidates set" and visualize them
    // TODO JONAS visualisierung ist wichtig! zeichne einfach die ersten 1000!
    // TODO smart way as usually are many
    Map<Tensor, Set<CandidatePair>> candidateMap = OptimalAnyTrajectoryPlanner.getCandidateMap();
  }
}
