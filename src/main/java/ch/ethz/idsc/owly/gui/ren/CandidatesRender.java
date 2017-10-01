// code by jl
package ch.ethz.idsc.owly.gui.ren;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import ch.ethz.idsc.owly.glc.core.CandidatePair;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.tensor.Tensor;

class CandidatesRender implements RenderInterface {
  private final OptimalAnyTrajectoryPlanner optimalAnyTrajectoryPlanner;

  CandidatesRender(OptimalAnyTrajectoryPlanner optimalAnyTrajectoryPlanner) {
    this.optimalAnyTrajectoryPlanner = optimalAnyTrajectoryPlanner;
  }

  @Override
  public void render(GeometricLayer owlyLayer, Graphics2D graphics) {
    graphics.setColor(Color.black);
    Map<Tensor, Set<CandidatePair>> candidateMap = optimalAnyTrajectoryPlanner.getCandidateMap();
    long candidateThreshold = -1; // Threshold, rendering the first ___ Candidates
    long iter = 0;
    Iterator<Set<CandidatePair>> candidateMapIterator = candidateMap.values().iterator();
    while (candidateMapIterator.hasNext()) {
      if (iter > candidateThreshold)
        break;
      Iterator<CandidatePair> candidateSetIterator = candidateMapIterator.next().iterator();
      while (candidateSetIterator.hasNext()) {
        if (iter > candidateThreshold)
          break;
        final CandidatePair candidate = candidateSetIterator.next();
        final Point2D p1 = owlyLayer.toPoint2D(candidate.getCandidate().state());
        final Point2D p2 = owlyLayer.toPoint2D(candidate.getOrigin().state());
        Stroke dashed = new BasicStroke(//
            (float) 1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[] { 5 }, 0);
        graphics.setStroke(dashed);
        graphics.setColor(new Color(0, 0, 0, 30));
        Shape shape = new Line2D.Double(p2.getX(), p2.getY(), p1.getX(), p1.getY());
        graphics.draw(shape);
        iter++;
      }
    }
  }
}
