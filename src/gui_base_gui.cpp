#include "gui_base_gui.hpp"

#include <cstdio>

#include "auton.hpp"
#include "api.h"

namespace aon {
namespace gui {

static constexpr int BRAIN_W = 480;
static constexpr int BRAIN_H = 240;

static pros::Task* g_task = nullptr;
static volatile bool g_run = false;

// Screen state
enum Screen { kMainMenu, kSelectRed, kSelectBlue, kSelectSkill, kDebugging };
static Screen CurrentScreen = kMainMenu;
static Screen PreviousScreen = kMainMenu;
static bool drawn = false;

static pros::screen_touch_status_s_t TouchStatus;

// 3 columns, 4 rows layout
static int blocks_x[] = {0, BRAIN_W / 3, BRAIN_W - BRAIN_W / 3, BRAIN_W};
static int blocks_y[] = {0, BRAIN_H / 4, BRAIN_H / 2, BRAIN_H - BRAIN_H / 4, BRAIN_H};

static const int menu_block_x = blocks_x[1] - 60;
static const int menu_block_y = blocks_y[1];

static const int menu_text_x = BRAIN_W / 5 - BRAIN_W / 10 - 40;
static const int menu_text_y = menu_block_y - BRAIN_H / 6;

static const int lower_block_text_y = BRAIN_H - BRAIN_H / 6;
static const int lower_block_1_text_x = BRAIN_W / 6 - BRAIN_W / 12;
static const int lower_block_2_text_x = BRAIN_W / 2 - BRAIN_W / 12;
static const int lower_block_3_text_x = BRAIN_W - BRAIN_W / 4;

// Selection state
static AutonId g_selected = AutonId::Right;

AutonId GetSelectedAuton() { return g_selected; }
const char* GetSelectedName() { return auton_name(g_selected); }

static void DrawButtonBlock(const std::uint32_t block_color,
                            int x1, int y1, int x2, int y2,
                            const std::uint32_t text_color,
                            pros::text_format_e_t fmt,
                            int tx, int ty,
                            const std::string& text) {
  pros::screen::set_eraser(block_color);
  pros::screen::erase_rect(x1, y1, x2, y2);
  pros::screen::set_pen(text_color);
  pros::screen::print(fmt, tx, ty, "%s", text.c_str());
}

static void DrawPressedBlock(int x1, int y1, int x2, int y2) {
  pros::screen::set_eraser(pros::c::COLOR_GRAY);
  pros::screen::erase_rect(x1, y1, x2, y2);
}

static void DrawHeader(const std::uint32_t color, const char* title) {
  pros::screen::set_pen(color);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "%s", title);

  // Small status line with selected auton
  pros::screen::set_pen(pros::c::COLOR_WHITE_SMOKE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 10, 30, "Selected: %s", auton_name(g_selected));
}

// Minimal "logo" placeholder.
// If you want a real bitmap logo on the brain, see the section at the bottom.
static void DrawLogoText() {
  pros::screen::set_pen(pros::c::COLOR_WHITE_SMOKE);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5, "2270A  RUST-EZE");
}

static void DrawCurrentScreen() {
  pros::screen::set_eraser(pros::c::COLOR_BLACK);
  pros::screen::erase();

  DrawLogoText();

  switch (CurrentScreen) {
    case kMainMenu: {
      DrawHeader(pros::c::COLOR_GREEN, "MAIN MENU");

      DrawButtonBlock(pros::c::COLOR_RED,
                      blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4],
                      pros::c::COLOR_WHITE_SMOKE, pros::E_TEXT_LARGE_CENTER,
                      lower_block_1_text_x, lower_block_text_y, "RED");

      DrawButtonBlock(pros::c::COLOR_BLUE,
                      blocks_x[1], blocks_y[3], blocks_x[2], blocks_y[4],
                      pros::c::COLOR_WHITE_SMOKE, pros::E_TEXT_LARGE_CENTER,
                      lower_block_2_text_x, lower_block_text_y, "BLUE");

      DrawButtonBlock(pros::c::COLOR_GREEN,
                      blocks_x[2], blocks_y[3], blocks_x[3], blocks_y[4],
                      pros::c::COLOR_WHITE_SMOKE, pros::E_TEXT_LARGE_CENTER,
                      lower_block_3_text_x - 20, lower_block_text_y, "SKILLS");

      DrawButtonBlock(pros::c::COLOR_LIGHT_STEEL_BLUE,
                      blocks_x[2] + 20, blocks_y[0], blocks_x[3], blocks_y[1],
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      blocks_x[2] + 40, blocks_y[1] - BRAIN_H / 6, "DEBUG");
    } break;

    case kSelectRed: {
      DrawHeader(pros::c::COLOR_RED, "SELECT RED");

      DrawButtonBlock(pros::c::COLOR_YELLOW,
                      blocks_x[0], blocks_y[0], menu_block_x, menu_block_y,
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE, menu_text_x, menu_text_y, "MENU");
      // Map RED page to match style picks
      DrawButtonBlock(pros::c::COLOR_LIGHT_PINK,
                      blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4],
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      lower_block_1_text_x, lower_block_text_y, "RIGHT");

      DrawButtonBlock(pros::c::COLOR_CRIMSON,
                      blocks_x[1], blocks_y[3], blocks_x[2], blocks_y[4],
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      lower_block_2_text_x, lower_block_text_y, "KEVIN");

      DrawButtonBlock(pros::c::COLOR_RED,
                      blocks_x[2], blocks_y[3], blocks_x[3], blocks_y[4],
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      lower_block_3_text_x - 30, lower_block_text_y, "SK60");
    } break;

    case kSelectBlue: {
      DrawHeader(pros::c::COLOR_BLUE, "SELECT BLUE");

      DrawButtonBlock(pros::c::COLOR_YELLOW,
                      blocks_x[0], blocks_y[0], menu_block_x, menu_block_y,
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE, menu_text_x, menu_text_y, "MENU");
      // Map BLUE page to the other side
      DrawButtonBlock(pros::c::COLOR_SKY_BLUE,
                      blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4],
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      lower_block_1_text_x, lower_block_text_y, "LEFT");

      DrawButtonBlock(pros::c::COLOR_STEEL_BLUE,
                      blocks_x[1], blocks_y[3], blocks_x[2], blocks_y[4],
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      lower_block_2_text_x, lower_block_text_y, "KEVIN");

      DrawButtonBlock(pros::c::COLOR_BLUE,
                      blocks_x[2], blocks_y[3], blocks_x[3], blocks_y[4],
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      lower_block_3_text_x - 30, lower_block_text_y, "SK60");
    } break;

    case kSelectSkill: {
      DrawHeader(pros::c::COLOR_GREEN, "SELECT SKILLS");

      DrawButtonBlock(pros::c::COLOR_YELLOW,
                      blocks_x[0], blocks_y[0], menu_block_x, menu_block_y,
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE, menu_text_x, menu_text_y, "MENU");

      DrawButtonBlock(pros::c::COLOR_GREEN,
                      blocks_x[0], blocks_y[3], blocks_x[3], blocks_y[4],
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      30, lower_block_text_y, "KEVIN SKILLS");
    } break;

    case kDebugging: {
      DrawHeader(pros::c::COLOR_GRAY, "DEBUG");

      DrawButtonBlock(pros::c::COLOR_YELLOW,
                      blocks_x[0], blocks_y[0], menu_block_x, menu_block_y,
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE, menu_text_x, menu_text_y, "MENU");
      // Keep these as placeholders until you send your test function names.
      DrawButtonBlock(pros::c::COLOR_SKY_BLUE,
                      blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4],
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      lower_block_1_text_x, lower_block_text_y, "N/A");

      DrawButtonBlock(pros::c::COLOR_LIGHT_STEEL_BLUE,
                      blocks_x[1], blocks_y[3], blocks_x[2], blocks_y[4],
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      lower_block_2_text_x, lower_block_text_y, "N/A");

      DrawButtonBlock(pros::c::COLOR_BLUE,
                      blocks_x[2], blocks_y[3], blocks_x[3], blocks_y[4],
                      pros::c::COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      lower_block_3_text_x, lower_block_text_y, "N/A");
    } break;

    default:
      break;
  }

  drawn = true;
}

static void ApplySelection(AutonId id) {
  g_selected = id;
  g_auton_selected = id;

  // Keep auton_kevin mode consistent with your dispatcher
  if (id == AutonId::Auton_Kevin_IZQ) {
    auton_kevin::set_mode(auton_kevin::Mode::MATCH_IZQ);
  } else if (id == AutonId::Auton_Kevin_SKILLS_IZQ) {
    auton_kevin::set_mode(auton_kevin::Mode::SKILLS_IZQ);
  }

  // Optional: store a callable in the reader too
  AutonomousReader->AddFunction("autonomous", []() -> int {
    autonomous_routine();
    return 0;
  });
}

static void HandleButtonPress() {
  PreviousScreen = CurrentScreen;

  switch (CurrentScreen) {
    case kMainMenu: {
      if (TouchStatus.y > blocks_y[3]) {
        if (TouchStatus.x < blocks_x[1]) {
          DrawPressedBlock(blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4]);
          CurrentScreen = kSelectRed;
        } else if (TouchStatus.x < blocks_x[2]) {
          DrawPressedBlock(blocks_x[1], blocks_y[3], blocks_x[2], blocks_y[4]);
          CurrentScreen = kSelectBlue;
        } else {
          DrawPressedBlock(blocks_x[2], blocks_y[3], blocks_x[3], blocks_y[4]);
          CurrentScreen = kSelectSkill;
        }
        pros::delay(180);
      } else if (TouchStatus.y < blocks_y[1] && TouchStatus.x > blocks_x[2]) {
        DrawPressedBlock(blocks_x[2], blocks_y[0], blocks_x[3], blocks_y[1]);
        CurrentScreen = kDebugging;
        pros::delay(180);
      }
    } break;

    case kSelectRed: {
      if (TouchStatus.y < menu_block_y && TouchStatus.y > blocks_y[0] && TouchStatus.x < menu_block_x) {
        DrawPressedBlock(blocks_x[0], blocks_y[0], menu_block_x, menu_block_y);
        CurrentScreen = kMainMenu;
        pros::delay(180);
      } else if (TouchStatus.y > blocks_y[3]) {
        if (TouchStatus.x < blocks_x[1]) {
          DrawPressedBlock(blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4]);
          ApplySelection(AutonId::Right);
        } else if (TouchStatus.x < blocks_x[2]) {
          DrawPressedBlock(blocks_x[1], blocks_y[3], blocks_x[2], blocks_y[4]);
          ApplySelection(AutonId::Auton_Kevin_IZQ);
        } else {
        }
        pros::delay(180);
      }
    } break;

    case kSelectBlue: {
      if (TouchStatus.y < menu_block_y && TouchStatus.y > blocks_y[0] && TouchStatus.x < menu_block_x) {
        DrawPressedBlock(blocks_x[0], blocks_y[0], menu_block_x, menu_block_y);
        CurrentScreen = kMainMenu;
        pros::delay(180);
      } else if (TouchStatus.y > blocks_y[3]) {
        if (TouchStatus.x < blocks_x[1]) {
          DrawPressedBlock(blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4]);
          ApplySelection(AutonId::Left);
        } else if (TouchStatus.x < blocks_x[2]) {
          DrawPressedBlock(blocks_x[1], blocks_y[3], blocks_x[2], blocks_y[4]);
          ApplySelection(AutonId::Auton_Kevin_IZQ);
        } else {
        }
        pros::delay(180);
      }
    } break;

    case kSelectSkill: {
      if (TouchStatus.y < menu_block_y && TouchStatus.y > blocks_y[0] && TouchStatus.x < menu_block_x) {
        DrawPressedBlock(blocks_x[0], blocks_y[0], menu_block_x, menu_block_y);
        CurrentScreen = kMainMenu;
        pros::delay(180);
      } else if (TouchStatus.y > blocks_y[3]) {
        DrawPressedBlock(blocks_x[0], blocks_y[3], blocks_x[3], blocks_y[4]);
        ApplySelection(AutonId::Auton_Kevin_SKILLS_IZQ);
        pros::delay(180);
      }
    } break;

    case kDebugging: {
      if (TouchStatus.y < menu_block_y && TouchStatus.y > blocks_y[0] && TouchStatus.x < menu_block_x) {
        DrawPressedBlock(blocks_x[0], blocks_y[0], menu_block_x, menu_block_y);
        CurrentScreen = kMainMenu;
        pros::delay(180);
      }
    } break;

    default:
      break;
  }

  if (PreviousScreen != CurrentScreen) drawn = false;
}

static void GuiTaskFn(void*) {
  pros::delay(20);

  pros::screen::set_eraser(pros::c::COLOR_BLACK);
  pros::screen::erase();
  pros::screen::set_pen(pros::c::COLOR_WHITE_SMOKE);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "Starting GUI");
  pros::delay(400);

  while (g_run) {
    if (!drawn) DrawCurrentScreen();

    TouchStatus = pros::screen::touch_status();
    if (TouchStatus.touch_status > 0) HandleButtonPress();

    pros::delay(20);
  }
}

void StartBaseGui() {
  if (g_task != nullptr) return;
  g_run = true;
  g_task = new pros::Task(GuiTaskFn, nullptr, "Brain GUI");
}

void Stop() {
  g_run = false;
  pros::delay(50);
  if (g_task) {
    delete g_task;
    g_task = nullptr;
  }
}

}  // namespace gui
}  // namespace aon
