// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Graphics2D;

interface AbstractRender {
  void render(OwlyLayer abstractLayer, Graphics2D graphics);
}
