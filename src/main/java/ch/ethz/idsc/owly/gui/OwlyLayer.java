// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.List;
import java.util.function.Function;

import ch.ethz.idsc.owly.math.Se2Utils;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;

public final class OwlyLayer {
  private static final double WHEEL_ANGLE = Math.PI / 10;
  // ---
  private final Function<Tensor, Point2D> toPoint2D_function;
  private Tensor mouseLocation = Array.zeros(2);
  private int mouseWheel = 0;

  OwlyLayer(Function<Tensor, Point2D> function) {
    this.toPoint2D_function = function;
  }

  /** only the first 2 entries of x are taken into account
   * 
   * @param x = {px, py, ...}
   * @return */
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

  /** @param location of mouse in model coordinates */
  /* package */ void setMouseLocation(Tensor location) {
    mouseLocation = location;
  }

  /** @return location of mouse in model coordinates */
  // public Tensor getMouseLocation() {
  // return mouseLocation.copy();
  // }
  /* package */ void incrementMouseWheel(int delta) {
    mouseWheel += delta;
  }

  /** @return absolute mouse wheel rotation since creation of component in model angle */
  /* package */ Scalar getMouseAngle() {
    return RealScalar.of(mouseWheel * WHEEL_ANGLE);
  }

  public Tensor getMouseSe2State() {
    return Tensors.of(mouseLocation.Get(0), mouseLocation.Get(1), getMouseAngle());
  }

  /** @return affine matrix that combines mouse location and mouse wheel rotation */
  public Tensor getMouseSe2Matrix() {
    return Se2Utils.toSE2Matrix(getMouseSe2State());
  }
}
