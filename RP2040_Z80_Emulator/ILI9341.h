#pragma once
void setup_tft(void) {
  tft.begin();

  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.println("RP2040 Z80 Emulator");  

  //tft.setCursor(0, 10);
  //tft.print(WiFi.localIP());
}

void update_tft(void) {
  if (dled == false) {
    uint8_t oled1, oled2;
    const int oled1_y = 49;
    const int oled2_y = 59;
    bool dledO;



    if (oled1 != pOut[0]) {
      if (bitRead(oled1, 0) != bitRead(pOut[0], 0)) {
        if (bitRead(pOut[0], 0) == true) {
          tft.fillCircle(90, oled1_y, 4, ILI9341_RED);
        } else {
          tft.fillCircle(90, oled1_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled1, 1) != bitRead(pOut[0], 1)) {
        if (bitRead(pOut[0], 1) == true) {
          tft.fillCircle(78, oled1_y, 4, ILI9341_RED);
        } else {
          tft.fillCircle(78, oled1_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled1, 2) != bitRead(pOut[0], 2)) {
        if (bitRead(pOut[0], 2) == true) {
          tft.fillCircle(66, oled1_y, 4, ILI9341_RED);
        } else {
          tft.fillCircle(66, oled1_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled1, 3) != bitRead(pOut[0], 3)) {
        if (bitRead(pOut[0], 3) == true) {
          tft.fillCircle(54, oled1_y, 4, ILI9341_RED);
        } else {
          tft.fillCircle(54, oled1_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled1, 4) != bitRead(pOut[0], 4)) {
        if (bitRead(pOut[0], 4) == true) {
          tft.fillCircle(42, oled1_y, 4, ILI9341_RED);
        } else {
          tft.fillCircle(42, oled1_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled1, 5) != bitRead(pOut[0], 5)) {
        if (bitRead(pOut[0], 5) == true) {
          tft.fillCircle(30, oled1_y, 4, ILI9341_RED);
        } else {
          tft.fillCircle(30, oled1_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled1, 6) != bitRead(pOut[0], 6)) {
        if (bitRead(pOut[0], 6) == true) {
          tft.fillCircle(18, oled1_y, 4, ILI9341_RED);
        } else {
          tft.fillCircle(18, oled1_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled1, 7) != bitRead(pOut[0], 7)) {
        if (bitRead(pOut[0], 7) == true) {
          tft.fillCircle(6, oled1_y, 4, ILI9341_RED);
        } else {
          tft.fillCircle(6, oled1_y, 4, ILI9341_BLACK);
        }
      }
      oled1 = pOut[0];
    }

    if (oled2 != pOut[2]) {
      if (bitRead(oled2, 0) != bitRead(pOut[2], 0)) {
        if (bitRead(pOut[2], 0) == true) {
          tft.fillCircle(90, oled2_y, 4, ILI9341_GREEN);
        } else {
          tft.fillCircle(90, oled2_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled2, 1) != bitRead(pOut[2], 1)) {
        if (bitRead(pOut[2], 1) == true) {
          tft.fillCircle(78, oled2_y, 4, ILI9341_GREEN);
        } else {
          tft.fillCircle(78, oled2_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled2, 2) != bitRead(pOut[2], 2)) {
        if (bitRead(pOut[2], 2) == true) {
          tft.fillCircle(66, oled2_y, 4, ILI9341_GREEN);
        } else {
          tft.fillCircle(66, oled2_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled2, 3) != bitRead(pOut[2], 3)) {
        if (bitRead(pOut[2], 3) == true) {
          tft.fillCircle(54, oled2_y, 4, ILI9341_GREEN);
        } else {
          tft.fillCircle(54, oled2_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled2, 4) != bitRead(pOut[2], 4)) {
        if (bitRead(pOut[2], 4) == true) {
          tft.fillCircle(42, oled2_y, 4, ILI9341_GREEN);
        } else {
          tft.fillCircle(42, oled2_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled2, 5) != bitRead(pOut[2], 5)) {
        if (bitRead(pOut[2], 5) == true) {
          tft.fillCircle(30, oled2_y, 4, ILI9341_GREEN);
        } else {
          tft.fillCircle(30, oled2_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled2, 6) != bitRead(pOut[2], 6)) {
        if (bitRead(pOut[2], 6) == true) {
          tft.fillCircle(18, oled2_y, 4, ILI9341_GREEN);
        } else {
          tft.fillCircle(18, oled2_y, 4, ILI9341_BLACK);
        }
      }
      if (bitRead(oled2, 7) != bitRead(pOut[2], 7)) {
        if (bitRead(pOut[2], 7) == true) {
          tft.fillCircle(6, oled2_y, 4, ILI9341_GREEN);
        } else {
          tft.fillCircle(6, oled2_y, 4, ILI9341_BLACK);
        }
      }
      oled2 = pOut[02];
    }

    if (dled != dledO) {
      dledO = dled;
      if (dled == true) {
        tft.fillCircle(6, 39, 4, ILI9341_BLUE);
      } else {
        tft.fillCircle(6, 39, 4, ILI9341_BLACK);
      }
    }
  }
}