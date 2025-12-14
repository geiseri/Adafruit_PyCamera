#pragma once

#include <Arduino.h>
#include <cstddef>
#include <array>
#include <esp_camera.h>
#include <Adafruit_AW9523.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SdFat_Adafruit_Fork.h>
#include <functional>
#include <JPEGDEC.h>
#ifndef TAG
#define TAG "PYCAM"
#endif

/**************************************************************************/
/**
 * @brief Framebuffer class for PyCamera.
 *
 * @details This class extends GFXcanvas16 to provide a framebuffer for the
 * PyCamera.
 */
/**************************************************************************/
class PyCameraFB : public GFXcanvas16 {
public:
  /**************************************************************************/
  /**
   * @brief Construct a new PyCameraFB object.
   *
   * @param w Width of the framebuffer.
   * @param h Height of the framebuffer.
   */
  /**************************************************************************/
  PyCameraFB(uint16_t w, uint16_t h) : GFXcanvas16(w, h) {

  };

  /**************************************************************************/
  /**
   * @brief Set the framebuffer.
   *
   * @param fb Pointer to the framebuffer data.
   */
  /**************************************************************************/
  void setFB(uint16_t *fb) { 
    auto old = buffer;
    buffer = fb;
    buffer_owned = false;
    free(old);
    }

};

/**************************************************************************/
/**
 * @brief Main class for Adafruit PyCamera.
 *
 * @details This class extends Adafruit_ST7789 and provides functionalities
 * for operating the PyCamera.
 */
/**************************************************************************/
class Adafruit_PyCamera : public Adafruit_ST7789 {
public:
  /**************************************************************************/
  /**
   * @brief Button mask values for AW9523 expander inputs.
   */
  /**************************************************************************/
  enum class ButtonMask : uint32_t {
    DOWN = (1UL << AWEXP_BUTTON_DOWN),
    LEFT = (1UL << AWEXP_BUTTON_LEFT),
    UP = (1UL << AWEXP_BUTTON_UP),
    RIGHT = (1UL << AWEXP_BUTTON_RIGHT),
    OK = (1UL << AWEXP_BUTTON_OK),
    SEL = (1UL << AWEXP_BUTTON_SEL),
    CARDDET = (1UL << AWEXP_SD_DET),
    INPUTS = (1UL << AWEXP_BUTTON_DOWN) | (1UL << AWEXP_BUTTON_LEFT) |
             (1UL << AWEXP_BUTTON_UP) | (1UL << AWEXP_BUTTON_RIGHT) |
             (1UL << AWEXP_BUTTON_OK) | (1UL << AWEXP_BUTTON_SEL) |
             (1UL << AWEXP_SD_DET)
  };

  /**
   * @brief Structure containing framesize information.
   */
  struct FramesizeInfo {
    framesize_t framesize;
    uint16_t width;
    uint16_t height;
    const char* name;
  };

  static constexpr std::array<FramesizeInfo, 25> Framesize = {{
    {FRAMESIZE_96X96, 96, 96, "96x96"},
    {FRAMESIZE_QQVGA, 160, 120, "QQVGA"},
    {FRAMESIZE_128X128, 128, 128, "128x128"},
    {FRAMESIZE_QCIF, 176, 144, "QCIF"},
    {FRAMESIZE_HQVGA, 240, 176, "HQVGA"},
    {FRAMESIZE_240X240, 240, 240, "240x240"},
    {FRAMESIZE_QVGA, 320, 240, "QVGA"},
    {FRAMESIZE_320X320, 320, 320, "320x320"},
    {FRAMESIZE_CIF, 400, 296, "CIF"},
    {FRAMESIZE_HVGA, 480, 320, "HVGA"},
    {FRAMESIZE_VGA, 640, 480, "VGA"},
    {FRAMESIZE_SVGA, 800, 600, "SVGA"},
    {FRAMESIZE_XGA, 1024, 768, "XGA"},
    {FRAMESIZE_HD, 1280, 720, "HD"},
    {FRAMESIZE_SXGA, 1280, 1024, "SXGA"},
    {FRAMESIZE_UXGA, 1600, 1200, "UXGA"},
    {FRAMESIZE_FHD, 1920, 1080, "FHD"},
    {FRAMESIZE_P_HD, 720, 1280, "P_HD"},
    {FRAMESIZE_P_3MP, 864, 1536, "P_3MP"},
    {FRAMESIZE_QXGA, 2048, 1536, "QXGA"},
    {FRAMESIZE_QHD, 2560, 1440, "QHD"},
    {FRAMESIZE_WQXGA, 2560, 1600, "WQXGA"},
    {FRAMESIZE_P_FHD, 1080, 1920, "P_FHD"},
    {FRAMESIZE_QSXGA, 2560, 1920, "QSXGA"},
    {FRAMESIZE_5MP, 2592, 1944, "5MP"},
  }};

  /**
   * @brief Structure containing special effect information.
   */
  struct SpecialEffectInfo {
    uint8_t effect;
    const char* name;
  };

  static constexpr std::array<SpecialEffectInfo, 7> SpecialEffect = {{
    {0, "Normal"},
    {1, "Negative"},
    {2, "Grayscale"},
    {3, "Red Tint"},
    {4, "Green Tint"},
    {5, "Blue Tint"},
    {6, "Sepia"},
  }};
  /**************************************************************************/
  /**
   * @brief Construct a new Adafruit_PyCamera object.
   */
  /**************************************************************************/
  Adafruit_PyCamera();

  bool setup(uint32_t freq = 0);

  bool initCamera(bool hwreset);
  bool initDisplay(void);
  bool initExpander(void);
  bool initAccel(void);
  bool initSD(void);
  void endSD(void);
  void I2Cscan(void);

  bool captureFrame(std::function<bool(camera_fb_t*)> hook);
  bool captureFrame(int x = 0, int y = 0, int width = 240, int height = 240);
  void blitFrame(void);
  bool takePhoto(const char *filename_base, framesize_t framesize);
  bool setFramesize(framesize_t framesize);
  framesize_t getFramesize();
  bool cycleFramesizeForward();
  bool cycleFramesizeBackward();
  framesize_t framesize_ = FRAMESIZE_UXGA;
  bool setSpecialEffect(uint8_t effect);
  uint8_t getSpecialEffect();
  bool cycleSpecialEffectForward();
  bool cycleSpecialEffectBackward();
  void speaker_tone(uint32_t tonefreq, uint32_t tonetime);

  float readBatteryVoltage(void);
  uint32_t readButtons(void);
  bool SDdetected(void);
  bool justReleased(uint8_t button_pin);
  bool justPressed(uint8_t button_pin);

  bool readAccelData(int16_t *x, int16_t *y, int16_t *z);
  bool readAccelData(float *x, float *y, float *z);

  void setNeopixel(uint32_t c);
  void setRing(uint32_t c);
  uint32_t Wheel(byte WheelPos);

  uint32_t timestamp(void);
  void timestampPrint(const char *msg);
  /** @brief Pointer to the camera sensor structure. */
  sensor_t *camera;
  /** @brief The PyCamera framebuffer object. */
  PyCameraFB *fb = nullptr;
  /** @brief Adafruit NeoPixel object for single pixel control. */
  Adafruit_NeoPixel pixel;
  /** @brief Adafruit NeoPixel object for ring control. */
  Adafruit_NeoPixel ring;
  /** @brief Adafruit AW9523 object for I/O expander functionality. */
  Adafruit_AW9523 aw;
  /** @brief Pointer to the I2C device for accelerometer. */
  Adafruit_I2CDevice *lis_dev = NULL;

  /** @brief SdFat object for SD card operations. */
  SdFat sd;
  /** @brief Last state of the buttons. */
  uint32_t last_button_state = 0xFFFFFFFF;
  /** @brief Current state of the buttons. */
  uint32_t button_state = 0xFFFFFFFF;

  /** @brief Current photo size setting. */
  //framesize_t photoSize = FRAMESIZE_VGA;
  /** @brief Current special effect setting. */
  //int8_t specialEffect = 0;
  /** @brief Configuration structure for the camera. */
  camera_config_t camera_config;

  // LIS3DH accelerometer register addresses
  static constexpr uint8_t LIS3DH_REG_STATUS1 = 0x07;
  static constexpr uint8_t LIS3DH_REG_OUTADC1_L = 0x08; /**< 1-axis acceleration data. Low value */
  static constexpr uint8_t LIS3DH_REG_OUTADC1_H = 0x09; /**< 1-axis acceleration data. High value */
  static constexpr uint8_t LIS3DH_REG_OUTADC2_L = 0x0A; /**< 2-axis acceleration data. Low value */
  static constexpr uint8_t LIS3DH_REG_OUTADC2_H = 0x0B; /**< 2-axis acceleration data. High value */
  static constexpr uint8_t LIS3DH_REG_OUTADC3_L = 0x0C; /**< 3-axis acceleration data. Low value */
  static constexpr uint8_t LIS3DH_REG_OUTADC3_H = 0x0D; /**< 3-axis acceleration data. High value */
  static constexpr uint8_t LIS3DH_REG_INTCOUNT = 0x0E; /**< INT_COUNTER register [IC7, IC6, IC5, IC4, IC3, IC2, IC1, IC0] */
  static constexpr uint8_t LIS3DH_REG_WHOAMI = 0x0F;
  static constexpr uint8_t LIS3DH_REG_TEMPCFG = 0x1F;
  static constexpr uint8_t LIS3DH_REG_CTRL1 = 0x20;
  static constexpr uint8_t LIS3DH_REG_CTRL2 = 0x21;
  static constexpr uint8_t LIS3DH_REG_CTRL3 = 0x22;
  static constexpr uint8_t LIS3DH_REG_CTRL4 = 0x23;
  static constexpr uint8_t LIS3DH_REG_CTRL5 = 0x24;
  static constexpr uint8_t LIS3DH_REG_CTRL6 = 0x25;
  static constexpr uint8_t LIS3DH_REG_STATUS2 = 0x27;
  static constexpr uint8_t LIS3DH_REG_OUT_X_L = 0x28; /**< X-axis acceleration data. Low value */
  static constexpr uint16_t LIS3DH_LSB16_TO_KILO_LSB10 = 6400;

private:
  /** @brief JPEGDEC object for decoding JPEG images. */
  JPEGDEC *jpeg_ = nullptr;
  /** @brief Timestamp for internal timing operations. */
  uint32_t _timestamp;

};
