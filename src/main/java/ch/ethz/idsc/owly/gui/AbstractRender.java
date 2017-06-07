// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Graphics2D;

interface AbstractRender {
  void render(OwlyLayer owlyLayer, Graphics2D graphics);
}
