#pragma once

namespace rlc_executive
{

/**
 * @brief Common interface implemented by operator-input frontends owned by the teleoperation controller.
 */
class TeleoperationFrontend
{
public:
  /// @brief Virtual destructor for polymorphic ownership.
  virtual ~TeleoperationFrontend() = default;

  /// @brief Returns whether the frontend is currently allowed to publish operator commands.
  virtual bool isEnabled() const noexcept = 0;

  /**
   * @brief Enables or disables operator command publication.
   * @param[in] enabled `true` to allow command publication, `false` to ignore input.
   */
  virtual void setEnabled(bool enabled) noexcept = 0;
};

}  // namespace rlc_executive
