// code by jl
package ch.ethz.idsc.owly.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.Collection;
import java.util.Iterator;
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
    // TODO JONAS optimalAnyTrajectoryPlanner."get candidates set" and visualize them
    // TODO JONAS visualisierung ist wichtig! zeichne einfach die ersten 1000!
    // TODO smart way as usually are many
    Map<Tensor, Set<CandidatePair>> candidateMap = OptimalAnyTrajectoryPlanner.getCandidateMap();
    long totalCandidates = candidateMap.values().parallelStream().flatMap(Collection::stream).count();
    long candidateThreshold = 1000; // Threshold, when candidates should not be rendered anymore
    if (totalCandidates < candidateThreshold) {
      Iterator<Set<CandidatePair>> candidateMapIterator = candidateMap.values().iterator();
      while (candidateMapIterator.hasNext()) {
        Iterator<CandidatePair> candidateSetIterator = candidateMapIterator.next().iterator();
        while (candidateSetIterator.hasNext()) {
          final CandidatePair candidate = candidateSetIterator.next();
          final Point2D p1 = owlyLayer.toPoint2D(candidate.getCandidate().state());
          final Point2D p2 = owlyLayer.toPoint2D(candidate.getOrigin().state());
          Stroke dashed = new BasicStroke(//
              (float) 1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[] { 5 }, 0);
          graphics.setStroke(dashed);
          graphics.setColor(new Color(0, 0, 0, 30));
          Shape shape = new Line2D.Double(p2.getX(), p2.getY(), p1.getX(), p1.getY());
          graphics.draw(shape);
        }
      }
    }
  }
}
