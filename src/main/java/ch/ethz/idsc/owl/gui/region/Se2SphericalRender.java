// code by jph
package ch.ethz.idsc.owl.gui.region;

import java.awt.Graphics2D;

import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owly.demo.se2.Se2SphericalRegion;

class Se2SphericalRender implements RenderInterface {
  private final Se2SphericalRegion se2SphericalRegion;

  public Se2SphericalRender(Se2SphericalRegion se2SphericalRegion) {
    this.se2SphericalRegion = se2SphericalRegion;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    EllipseRegionRender.of(se2SphericalRegion.sphericalRegion).render(geometricLayer, graphics);
  }
}
