#pragma once

#include <cstdint>
#include <array>
#include <utility>
#include <esp_camera.h>

/**
 * @file CameraTypes.h
 * @brief Type definitions for camera configuration settings.
 * 
 * This file contains all enum classes and struct definitions used for
 * camera configuration validation and representation.
 */

namespace detail {
  /**
   * @brief Helper template to deduce array size from initializer list.
   *
   * @details This template function creates a std::array from a variadic list of
   * arguments, automatically deducing the array size from the number of arguments.
   * Used to create static constexpr arrays for camera configuration validation.
   *
   * @tparam T The element type of the array.
   * @tparam U Variadic parameter pack for the initializer list elements.
   * @param u Variadic arguments to initialize the array.
   * @return A std::array<T, N> where N is the number of arguments.
   */
  template<typename T, typename... U>
  constexpr auto make_static_array(U&&... u) -> std::array<T, sizeof...(U)> {
      return {{std::forward<U>(u)...}};
  }
}

enum class ContrastLevel : int8_t {
  CONTRAST_LOWEST = -2,
  CONTRAST_LOW = -1,
  CONTRAST_NORMAL = 0,
  CONTRAST_HIGH = 1,
  CONTRAST_HIGHEST = 2
};

struct ContrastInfo {
  ContrastLevel level;
  const char* name;
};

enum class SaturationLevel : int8_t {
  SATURATION_LOWEST = -2,
  SATURATION_LOW = -1,
  SATURATION_NORMAL = 0,
  SATURATION_HIGH = 1,
  SATURATION_HIGHEST = 2
};

struct SaturationInfo {
  SaturationLevel level;
  const char* name;
};

enum class SharpnessLevel : int8_t {
  SHARPNESS_SOFTEST = -2,
  SHARPNESS_SOFT = -1,
  SHARPNESS_NORMAL = 0,
  SHARPNESS_SHARP = 1,
  SHARPNESS_SHARPEST = 2
};

struct SharpnessInfo {
  SharpnessLevel level;
  const char* name;
};

enum class WhiteBalanceMode : uint8_t {
  WB_AUTO = 0,
  WB_SUNNY = 1,
  WB_CLOUDY = 2,
  WB_OFFICE = 3,
  WB_HOME = 4
};

struct WbModeInfo {
  WhiteBalanceMode mode;
  const char* name;
};

struct FramesizeInfo {
  framesize_t framesize;
  uint16_t width;
  uint16_t height;
  const char* name;
};

enum class SpecialEffect : uint8_t {
  EFFECT_NORMAL = 0,
  EFFECT_NEGATIVE = 1,
  EFFECT_GRAYSCALE = 2,
  EFFECT_RED_TINT = 3,
  EFFECT_GREEN_TINT = 4,
  EFFECT_BLUE_TINT = 5,
  EFFECT_SEPIA = 6
};

struct SpecialEffectInfo {
  SpecialEffect effect;
  const char* name;
};

struct BrightnessInfo {
  int8_t level;
  const char* name;
};

struct DenoiseInfo {
  uint8_t level;
  const char* name;
};

inline constexpr auto Framesize = detail::make_static_array<FramesizeInfo>(
  FramesizeInfo{FRAMESIZE_96X96, 96, 96, "96x96"},
  FramesizeInfo{FRAMESIZE_QQVGA, 160, 120, "QQVGA"},
  FramesizeInfo{FRAMESIZE_128X128, 128, 128, "128x128"},
  FramesizeInfo{FRAMESIZE_QCIF, 176, 144, "QCIF"},
  FramesizeInfo{FRAMESIZE_HQVGA, 240, 176, "HQVGA"},
  FramesizeInfo{FRAMESIZE_240X240, 240, 240, "240x240"},
  FramesizeInfo{FRAMESIZE_QVGA, 320, 240, "QVGA"},
  FramesizeInfo{FRAMESIZE_320X320, 320, 320, "320x320"},
  FramesizeInfo{FRAMESIZE_CIF, 400, 296, "CIF"},
  FramesizeInfo{FRAMESIZE_HVGA, 480, 320, "HVGA"},
  FramesizeInfo{FRAMESIZE_VGA, 640, 480, "VGA"},
  FramesizeInfo{FRAMESIZE_SVGA, 800, 600, "SVGA"},
  FramesizeInfo{FRAMESIZE_XGA, 1024, 768, "XGA"},
  FramesizeInfo{FRAMESIZE_HD, 1280, 720, "HD"},
  FramesizeInfo{FRAMESIZE_SXGA, 1280, 1024, "SXGA"},
  FramesizeInfo{FRAMESIZE_UXGA, 1600, 1200, "UXGA"}
);

inline constexpr auto SpecialEffectList = detail::make_static_array<SpecialEffectInfo>(
  SpecialEffectInfo{SpecialEffect::EFFECT_NORMAL, "Normal"},
  SpecialEffectInfo{SpecialEffect::EFFECT_NEGATIVE, "Negative"},
  SpecialEffectInfo{SpecialEffect::EFFECT_GRAYSCALE, "Grayscale"},
  SpecialEffectInfo{SpecialEffect::EFFECT_RED_TINT, "Red Tint"},
  SpecialEffectInfo{SpecialEffect::EFFECT_GREEN_TINT, "Green Tint"},
  SpecialEffectInfo{SpecialEffect::EFFECT_BLUE_TINT, "Blue Tint"},
  SpecialEffectInfo{SpecialEffect::EFFECT_SEPIA, "Sepia"}
);

inline constexpr auto Brightness = detail::make_static_array<BrightnessInfo>(
  BrightnessInfo{-2, "-2"},
  BrightnessInfo{-1, "-1"},
  BrightnessInfo{0, "0"},
  BrightnessInfo{1, "1"},
  BrightnessInfo{2, "2"}
);

inline constexpr auto Contrast = detail::make_static_array<ContrastInfo>(
  ContrastInfo{ContrastLevel::CONTRAST_LOWEST, "Lowest"},
  ContrastInfo{ContrastLevel::CONTRAST_LOW, "Low"},
  ContrastInfo{ContrastLevel::CONTRAST_NORMAL, "Normal"},
  ContrastInfo{ContrastLevel::CONTRAST_HIGH, "High"},
  ContrastInfo{ContrastLevel::CONTRAST_HIGHEST, "Highest"}
);

inline constexpr auto Saturation = detail::make_static_array<SaturationInfo>(
  SaturationInfo{SaturationLevel::SATURATION_LOWEST, "Lowest"},
  SaturationInfo{SaturationLevel::SATURATION_LOW, "Low"},
  SaturationInfo{SaturationLevel::SATURATION_NORMAL, "Normal"},
  SaturationInfo{SaturationLevel::SATURATION_HIGH, "High"},
  SaturationInfo{SaturationLevel::SATURATION_HIGHEST, "Highest"}
);

inline constexpr auto Sharpness = detail::make_static_array<SharpnessInfo>(
  SharpnessInfo{SharpnessLevel::SHARPNESS_SOFTEST, "Softest"},
  SharpnessInfo{SharpnessLevel::SHARPNESS_SOFT, "Soft"},
  SharpnessInfo{SharpnessLevel::SHARPNESS_NORMAL, "Normal"},
  SharpnessInfo{SharpnessLevel::SHARPNESS_SHARP, "Sharp"},
  SharpnessInfo{SharpnessLevel::SHARPNESS_SHARPEST, "Sharpest"}
);

inline constexpr auto Denoise = detail::make_static_array<DenoiseInfo>(
  DenoiseInfo{0, "0"},
  DenoiseInfo{1, "1"},
  DenoiseInfo{2, "2"},
  DenoiseInfo{3, "3"},
  DenoiseInfo{4, "4"},
  DenoiseInfo{5, "5"},
  DenoiseInfo{6, "6"},
  DenoiseInfo{7, "7"},
  DenoiseInfo{8, "8"}
);

inline constexpr auto WbMode = detail::make_static_array<WbModeInfo>(
  WbModeInfo{WhiteBalanceMode::WB_AUTO, "Auto"},
  WbModeInfo{WhiteBalanceMode::WB_SUNNY, "Sunny"},
  WbModeInfo{WhiteBalanceMode::WB_CLOUDY, "Cloudy"},
  WbModeInfo{WhiteBalanceMode::WB_OFFICE, "Office"},
  WbModeInfo{WhiteBalanceMode::WB_HOME, "Home"}
);
