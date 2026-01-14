#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "api.h"
#include "auton.hpp"
#include "function_reader.hpp"

namespace aon {

// Stores selectable actions. We keep int return type to match your template style.
inline std::unique_ptr<FunctionReader<int>> AutonomousReader =
    std::make_unique<FunctionReader<int>>();

namespace gui {

// Public API used by main.cpp
void Start();
void Stop();

// Optional helpers
AutonId GetSelectedAuton();
const char* GetSelectedName();

}  // namespace gui
}  // namespace aon
