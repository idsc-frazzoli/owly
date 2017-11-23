// code by jph
package ch.ethz.idsc.owl.gui.region;

import java.awt.Graphics2D;
import java.awt.geom.Path2D;

import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.math.planar.CirclePoints;
import ch.ethz.idsc.owl.math.region.EllipsoidRegion;
import ch.ethz.idsc.owl.math.region.SphericalRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** visualize planar ellipse */
class EllipseRegionRender implements RenderInterface {
  /** @param ellipsoidRegion
   * @return */
  static RenderInterface of(EllipsoidRegion ellipsoidRegion) {
    Tensor radius = ellipsoidRegion.radius();
    return new EllipseRegionRender(ellipsoidRegion.center().extract(0, 2), radius.Get(0), radius.Get(1));
  }

  /** @param sphericalRegion
   * @return */
  static RenderInterface of(SphericalRegion sphericalRegion) {
    return new EllipseRegionRender(sphericalRegion.center().extract(0, 2), sphericalRegion.radius(), sphericalRegion.radius());
  }

  // ---
  private static final int RESOLUTION = 22;
  // ---
  private final Tensor polygon;

  private EllipseRegionRender(Tensor center, Scalar radiusX, Scalar radiusY) {
    GlobalAssert.that(center.length() == 2);
    polygon = Tensor.of(CirclePoints.elliptic(RESOLUTION, radiusX, radiusY) //
        .stream().map(row -> row.add(center)));
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Path2D path2D = geometricLayer.toPath2D(polygon);
    graphics.setColor(RegionRenders.COLOR);
    graphics.fill(path2D);
    graphics.setColor(RegionRenders.BOUNDARY);
    path2D.closePath();
    graphics.draw(path2D);
  }
}
