// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.Graphics2D;
import java.awt.geom.Point2D;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.tensor.Tensor;

abstract class AbstractLayer {
  final GlcComponent glcComponent;

  AbstractLayer(GlcComponent glcComponent) {
    this.glcComponent = glcComponent;
  }

  Point2D toPoint2D(Tensor x) {
    return glcComponent.toPoint2D(x);
  }

  abstract void render(Graphics2D graphics, TrajectoryPlanner trajectoryPlanner);
}
