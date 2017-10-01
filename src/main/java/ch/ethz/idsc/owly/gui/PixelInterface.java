// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.geom.Point2D;

import ch.ethz.idsc.tensor.Tensor;

public interface PixelInterface {
  Point2D toPoint2D(Tensor x);
}
