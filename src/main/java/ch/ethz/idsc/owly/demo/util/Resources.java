// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.io.File;

public enum Resources {
  ;
  // for files in repo
  public static File fileFromRepository(String string) {
    return new File("".getClass().getResource(string).getPath());
  }
}
