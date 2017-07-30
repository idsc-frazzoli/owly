// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.List;
import java.util.function.Function;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Tensor;

public final class OwlyLayer {
  private final Function<Tensor, Point2D> toPoint2D_function;

  OwlyLayer(Function<Tensor, Point2D> function) {
    this.toPoint2D_function = function;
  }

  public Point2D toPoint2D(Tensor x) {
    return toPoint2D_function.apply(x);
  }

  public Path2D toVector(Tensor x, Tensor dx) {
    x = x.extract(0, 2);
    dx = dx.extract(0, 2);
    Path2D path2d = new Path2D.Double();
    Point2D p1 = toPoint2D(x);
    Point2D p2 = toPoint2D(x.add(dx));
    path2d.moveTo(p1.getX(), p1.getY());
    path2d.lineTo(p2.getX(), p2.getY());
    return path2d;
  }

  public Path2D toPath2D(List<StateTime> trajectory) {
    Path2D path2d = new Path2D.Double();
    if (!trajectory.isEmpty()) {
      StateTime prev = trajectory.get(0);
      Point2D p0 = toPoint2D(prev.state());
      path2d.moveTo(p0.getX(), p0.getY());
      for (StateTime stateTime : trajectory.subList(1, trajectory.size())) {
        Point2D p1 = toPoint2D(stateTime.state());
        path2d.lineTo(p1.getX(), p1.getY());
      }
    }
    return path2d;
  }

  public Path2D toPath2D(Tensor polygon) {
    Path2D path2d = new Path2D.Double();
    {
      Point2D point2d = toPoint2D(polygon.get(0));
      path2d.moveTo(point2d.getX(), point2d.getY());
    }
    polygon.flatten(0) //
        .skip(1) // first coordinate already used in moveTo
        .map(this::toPoint2D) //
        .forEach(point2d -> path2d.lineTo(point2d.getX(), point2d.getY()));
    return path2d;
  }
}
