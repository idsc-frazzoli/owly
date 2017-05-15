// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.List;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Tensor;

abstract class AbstractLayer {
  final OwlyComponent glcComponent;

  AbstractLayer(OwlyComponent glcComponent) {
    this.glcComponent = glcComponent;
  }

  final Point2D toPoint2D(Tensor x) {
    return glcComponent.toPoint2D(x);
  }

  final Path2D toPath2D(List<StateTime> trajectory) {
    Path2D path2d = new Path2D.Double();
    if (!trajectory.isEmpty()) {
      StateTime prev = trajectory.get(0);
      Point2D p0 = toPoint2D(prev.x());
      path2d.moveTo(p0.getX(), p0.getY());
      for (StateTime stateTime : trajectory.subList(1, trajectory.size())) {
        Point2D p1 = toPoint2D(stateTime.x());
        path2d.lineTo(p1.getX(), p1.getY());
      }
    }
    return path2d;
  }

  abstract void render(Graphics2D graphics);
}
