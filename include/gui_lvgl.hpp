#pragma once

#include "auton.hpp"

namespace aon {
namespace gui {

// Starts LVGL GUI on its own task.
void Start();

// Optional helpers.
AutonId GetSelected();
bool IsConfirmed();

}  // namespace gui
}  // namespace aon
