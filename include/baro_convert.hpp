#include <cmath>

static constexpr float BASE_TEMPERATURE = 296.15;
static constexpr float K1               = 153.8462f;
static constexpr float K2               = 0.190259f;

namespace BaroCnv
{
  float baro_pressure_to_height(  const double current_pressure,
                                  const double base_pressure )
  {
    // This is an exact calculation that is within +-2.5m of the standard
    // atmosphere tables in the troposphere (up to 11,000 m amsl).
    return K1 * BASE_TEMPERATURE
          * (1.0f - expf(K2 * logf(static_cast<float>(current_pressure / base_pressure))));
  }

  float baro_height_to_base_pressure( const double height,
                                      const double current_pressure )
  {
    return static_cast<float>(current_pressure)
          / (expf(logf(1.0f - static_cast<float>(height) / (K1 * BASE_TEMPERATURE)) / K2));
  }
}