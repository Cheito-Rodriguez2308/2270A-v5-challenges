#include "gui_lvgl.hpp"

#include "auton.hpp"
#include "gui_assets.hpp"   // extern const lv_image_dsc_t rust_eze_logo_200x200;
#include "liblvgl/lvgl.h"
#include "pros/rtos.hpp"
#include "liblvgl/lv_conf.h"

#include "api.h"

namespace aon {
namespace gui {

static std::atomic<AutonId> g_selected{AutonId::Right};
static std::atomic<bool>   g_confirmed{false};

AutonId GetSelected() { return g_selected.load(); }
bool IsConfirmed() { return g_confirmed.load(); }

// ---------------------------
// UI state
// ---------------------------
enum class ScreenId { Main, Red, Blue, Skills, Debug };
static ScreenId g_screen = ScreenId::Main;

// Pending selection before confirm
static AutonId g_pending = AutonId::Right;

// Root screens
static lv_obj_t* scr_main   = nullptr;
static lv_obj_t* scr_red    = nullptr;
static lv_obj_t* scr_blue   = nullptr;
static lv_obj_t* scr_skills = nullptr;
static lv_obj_t* scr_debug  = nullptr;

// Labels that show selection on each menu
static lv_obj_t* lbl_red_pick    = nullptr;
static lv_obj_t* lbl_blue_pick   = nullptr;
static lv_obj_t* lbl_skills_pick = nullptr;

// Popup objects
static lv_obj_t* popup_bg   = nullptr;
static lv_obj_t* popup_box  = nullptr;
static lv_obj_t* popup_lbl  = nullptr;

// ---------------------------
// Style helpers (LVGL v9)
// ---------------------------
static void set_bg_black(lv_obj_t* o) {
  lv_obj_set_style_bg_color(o, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, LV_PART_MAIN);
}

static void set_text_color(lv_obj_t* o, lv_color_t c) {
  lv_obj_set_style_text_color(o, c, LV_PART_MAIN);
}

static const char* auton_name_ui(AutonId id) {
  return auton_name(id);
}

// ---------------------------
// Screen switching
// ---------------------------
static void load(ScreenId id) {
  g_screen = id;
  switch (id) {
    case ScreenId::Main:   lv_screen_load(scr_main);   break;
    case ScreenId::Red:    lv_screen_load(scr_red);    break;
    case ScreenId::Blue:   lv_screen_load(scr_blue);   break;
    case ScreenId::Skills: lv_screen_load(scr_skills); break;
    case ScreenId::Debug:  lv_screen_load(scr_debug);  break;
  }
}

// ---------------------------
// Popup (LVGL v9 events)
// ---------------------------
static void popup_close() {
  if (popup_bg) {
    lv_obj_delete(popup_bg);
    popup_bg = nullptr;
    popup_box = nullptr;
    popup_lbl = nullptr;
  }
}

static void popup_cancel_cb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  popup_close();
}

static void popup_confirm_cb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  g_selected.store(g_pending);
  g_confirmed.store(true);

  // Your auton dispatcher variable
  g_auton_selected = g_pending;

  popup_close();
  load(ScreenId::Main);
}

static void popup_show_confirm(const char* text) {
  popup_close();

  lv_obj_t* parent = lv_screen_active();

  popup_bg = lv_obj_create(parent);
  lv_obj_set_size(popup_bg, 480, 240);
  lv_obj_set_pos(popup_bg, 0, 0);
  lv_obj_set_style_bg_color(popup_bg, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(popup_bg, LV_OPA_60, LV_PART_MAIN);
  lv_obj_set_style_border_width(popup_bg, 0, LV_PART_MAIN);

  popup_box = lv_obj_create(popup_bg);
  lv_obj_set_size(popup_box, 360, 150);
  lv_obj_center(popup_box);
  lv_obj_set_style_radius(popup_box, 14, LV_PART_MAIN);
  lv_obj_set_style_bg_color(popup_box, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(popup_box, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(popup_box, 2, LV_PART_MAIN);
  lv_obj_set_style_border_color(popup_box, lv_color_black(), LV_PART_MAIN);

  popup_lbl = lv_label_create(popup_box);
  lv_label_set_text(popup_lbl, text);
  lv_obj_align(popup_lbl, LV_ALIGN_TOP_MID, 0, 16);
  set_text_color(popup_lbl, lv_color_black());

  lv_obj_t* btn_cancel = lv_button_create(popup_box);
  lv_obj_set_size(btn_cancel, 140, 45);
  lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_LEFT, 20, -18);
  lv_obj_add_event_cb(btn_cancel, popup_cancel_cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* lbl_cancel = lv_label_create(btn_cancel);
  lv_label_set_text(lbl_cancel, "CANCEL");
  lv_obj_center(lbl_cancel);

  lv_obj_t* btn_ok = lv_button_create(popup_box);
  lv_obj_set_size(btn_ok, 140, 45);
  lv_obj_align(btn_ok, LV_ALIGN_BOTTOM_RIGHT, -20, -18);
  lv_obj_add_event_cb(btn_ok, popup_confirm_cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* lbl_ok = lv_label_create(btn_ok);
  lv_label_set_text(lbl_ok, "CONFIRM");
  lv_obj_center(lbl_ok);
}

// ---------------------------
// Button helpers
// ---------------------------
static lv_obj_t* make_big_btn(lv_obj_t* parent, lv_color_t bg, const char* text) {
  lv_obj_t* b = lv_button_create(parent);
  lv_obj_set_style_bg_color(b, bg, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(b, 0, LV_PART_MAIN);

  lv_obj_t* l = lv_label_create(b);
  lv_label_set_text(l, text);
  set_text_color(l, lv_color_white());
  lv_obj_center(l);

  return b;
}

static lv_obj_t* make_menu_btn(lv_obj_t* parent) {
  lv_obj_t* b = lv_button_create(parent);
  lv_obj_set_style_bg_color(b, lv_color_make(230, 200, 40), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_MAIN);

  lv_obj_t* l = lv_label_create(b);
  lv_label_set_text(l, "MENU");
  set_text_color(l, lv_color_black());
  lv_obj_center(l);

  return b;
}

static lv_obj_t* make_pick_btn(lv_obj_t* parent, lv_color_t bg, const char* text) {
  lv_obj_t* b = lv_button_create(parent);
  lv_obj_set_style_bg_color(b, bg, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_MAIN);

  lv_obj_t* l = lv_label_create(b);
  lv_label_set_text(l, text);
  set_text_color(l, lv_color_black());
  lv_obj_center(l);

  return b;
}

// ---------------------------
// Common selection helpers
// ---------------------------
static void set_pick_label(lv_obj_t* lbl, AutonId pick) {
  if (!lbl) return;
  std::string s = "Selected: ";
  s += auton_name_ui(pick);
  lv_label_set_text(lbl, s.c_str());
}

static void open_confirm_popup(AutonId pick) {
  g_pending = pick;
  char buf[96];
  std::snprintf(buf, sizeof(buf), "Confirm auton:\n%s", auton_name_ui(pick));
  popup_show_confirm(buf);
}

// ---------------------------
// Event callbacks (LVGL v9)
// ---------------------------
static void on_main_red(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  load(ScreenId::Red);
}

static void on_main_blue(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  load(ScreenId::Blue);
}

static void on_main_skills(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  load(ScreenId::Skills);
}

static void on_main_debug(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  load(ScreenId::Debug);
}

static void on_menu_back(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  load(ScreenId::Main);
}

// Red menu picks
static void on_pick_right(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  set_pick_label(lbl_red_pick, AutonId::Right);
  open_confirm_popup(AutonId::Right);
}
static void on_pick_left(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  set_pick_label(lbl_red_pick, AutonId::Left);
  open_confirm_popup(AutonId::Left);
}
static void on_pick_kevin_match(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  set_pick_label(lbl_red_pick, AutonId::Auton_Kevin_IZQ);
  open_confirm_popup(AutonId::Auton_Kevin_IZQ);
}
static void on_pick_kevin_skills(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  set_pick_label(lbl_red_pick, AutonId::Auton_Kevin_SKILLS_IZQ);
  open_confirm_popup(AutonId::Auton_Kevin_SKILLS_IZQ);
}

// Blue menu picks
static void on_pick_right_blue(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  set_pick_label(lbl_blue_pick, AutonId::Right);
  open_confirm_popup(AutonId::Right);
}
static void on_pick_left_blue(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  set_pick_label(lbl_blue_pick, AutonId::Left);
  open_confirm_popup(AutonId::Left);
}
static void on_pick_kevin_match_blue(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  set_pick_label(lbl_blue_pick, AutonId::Auton_Kevin_IZQ);
  open_confirm_popup(AutonId::Auton_Kevin_IZQ);
}
static void on_pick_kevin_skills_blue(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  set_pick_label(lbl_blue_pick, AutonId::Auton_Kevin_SKILLS_IZQ);
  open_confirm_popup(AutonId::Auton_Kevin_SKILLS_IZQ);
}

// Skills picks
static void on_pick_skills_kevin(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  set_pick_label(lbl_skills_pick, AutonId::Auton_Kevin_SKILLS_IZQ);
  open_confirm_popup(AutonId::Auton_Kevin_SKILLS_IZQ);
}

// ---------------------------
// Screen builders (LVGL v9)
// ---------------------------
static void build_main() {
  scr_main = lv_obj_create(nullptr);
  set_bg_black(scr_main);

  lv_obj_t* title = lv_label_create(scr_main);
  lv_label_set_text(title, "MAIN MENU");
  set_text_color(title, lv_color_make(0, 255, 120));
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t* img = lv_image_create(scr_main);
  lv_image_set_src(img, &rust_eze_logo_200x200);
  lv_obj_align(img, LV_ALIGN_CENTER, 0, -10);

  const int btn_h = 70;
  const int btn_y = 240 - btn_h;

  lv_obj_t* b_red = make_big_btn(scr_main, lv_color_make(220, 30, 30), "RED");
  lv_obj_set_size(b_red, 160, btn_h);
  lv_obj_set_pos(b_red, 0, btn_y);
  lv_obj_add_event_cb(b_red, on_main_red, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* b_blue = make_big_btn(scr_main, lv_color_make(30, 90, 220), "BLUE");
  lv_obj_set_size(b_blue, 160, btn_h);
  lv_obj_set_pos(b_blue, 160, btn_y);
  lv_obj_add_event_cb(b_blue, on_main_blue, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* b_sk = make_big_btn(scr_main, lv_color_make(0, 160, 90), "SKILLS");
  lv_obj_set_size(b_sk, 160, btn_h);
  lv_obj_set_pos(b_sk, 320, btn_y);
  lv_obj_add_event_cb(b_sk, on_main_skills, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* b_dbg = lv_button_create(scr_main);
  lv_obj_set_size(b_dbg, 120, 50);
  lv_obj_set_pos(b_dbg, 480 - 120, 0);
  lv_obj_set_style_bg_color(b_dbg, lv_color_make(160, 180, 210), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(b_dbg, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(b_dbg, on_main_debug, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* dbg_lbl = lv_label_create(b_dbg);
  lv_label_set_text(dbg_lbl, "DEBUG");
  set_text_color(dbg_lbl, lv_color_black());
  lv_obj_center(dbg_lbl);

  lv_obj_t* sel = lv_label_create(scr_main);
  std::string s = "Selected: ";
  s += auton_name_ui(g_selected.load());
  lv_label_set_text(sel, s.c_str());
  set_text_color(sel, lv_color_white());
  lv_obj_align(sel, LV_ALIGN_BOTTOM_MID, 0, -80);
}

static void build_pick_screen(lv_obj_t** out_scr,
                              lv_obj_t** out_lbl_pick,
                              const char* title_text,
                              lv_color_t title_color,
                              lv_color_t b1,
                              lv_color_t b2,
                              lv_color_t b3,
                              lv_color_t b4,
                              lv_event_cb_t cb1,
                              lv_event_cb_t cb2,
                              lv_event_cb_t cb3,
                              lv_event_cb_t cb4) {
  lv_obj_t* scr = lv_obj_create(nullptr);
  set_bg_black(scr);

  lv_obj_t* title = lv_label_create(scr);
  lv_label_set_text(title, title_text);
  set_text_color(title, title_color);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t* menu = make_menu_btn(scr);
  lv_obj_set_size(menu, 120, 50);
  lv_obj_set_pos(menu, 0, 0);
  lv_obj_add_event_cb(menu, on_menu_back, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* pick = lv_label_create(scr);
  lv_label_set_text(pick, "Selected: NONE");
  set_text_color(pick, lv_color_white());
  lv_obj_align(pick, LV_ALIGN_TOP_MID, 0, 65);

  const int w = 240;
  const int h = 80;
  const int y0 = 240 - 160;

  lv_obj_t* a = make_pick_btn(scr, b1, "RIGHT");
  lv_obj_set_size(a, w, h);
  lv_obj_set_pos(a, 0, y0);
  lv_obj_add_event_cb(a, cb1, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* b = make_pick_btn(scr, b2, "LEFT");
  lv_obj_set_size(b, w, h);
  lv_obj_set_pos(b, 240, y0);
  lv_obj_add_event_cb(b, cb2, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* c = make_pick_btn(scr, b3, "KEVIN MATCH");
  lv_obj_set_size(c, w, h);
  lv_obj_set_pos(c, 0, y0 + 80);
  lv_obj_add_event_cb(c, cb3, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* d = make_pick_btn(scr, b4, "KEVIN SKILLS");
  lv_obj_set_size(d, w, h);
  lv_obj_set_pos(d, 240, y0 + 80);
  lv_obj_add_event_cb(d, cb4, LV_EVENT_CLICKED, nullptr);

  *out_scr = scr;
  *out_lbl_pick = pick;
}

static void build_skills() {
  scr_skills = lv_obj_create(nullptr);
  set_bg_black(scr_skills);

  lv_obj_t* title = lv_label_create(scr_skills);
  lv_label_set_text(title, "SELECT SKILLS");
  set_text_color(title, lv_color_make(0, 255, 120));
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t* menu = make_menu_btn(scr_skills);
  lv_obj_set_size(menu, 120, 50);
  lv_obj_set_pos(menu, 0, 0);
  lv_obj_add_event_cb(menu, on_menu_back, LV_EVENT_CLICKED, nullptr);

  lbl_skills_pick = lv_label_create(scr_skills);
  lv_label_set_text(lbl_skills_pick, "Selected: NONE");
  set_text_color(lbl_skills_pick, lv_color_white());
  lv_obj_align(lbl_skills_pick, LV_ALIGN_TOP_MID, 0, 65);

  lv_obj_t* b2 = make_pick_btn(scr_skills, lv_color_make(170, 200, 230), "KEVIN SKILLS");
  lv_obj_set_size(b2, 480, 80);
  lv_obj_set_pos(b2, 0, 160);
  lv_obj_add_event_cb(b2, on_pick_skills_kevin, LV_EVENT_CLICKED, nullptr);
}

static void build_debug() {
  scr_debug = lv_obj_create(nullptr);
  set_bg_black(scr_debug);

  lv_obj_t* title = lv_label_create(scr_debug);
  lv_label_set_text(title, "DEBUG");
  set_text_color(title, lv_color_make(200, 200, 200));
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t* menu = make_menu_btn(scr_debug);
  lv_obj_set_size(menu, 120, 50);
  lv_obj_set_pos(menu, 0, 0);
  lv_obj_add_event_cb(menu, on_menu_back, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* msg = lv_label_create(scr_debug);
  lv_label_set_text(msg, "Place debug widgets here later.");
  set_text_color(msg, lv_color_white());
  lv_obj_center(msg);
}

static void build_all() {
  build_main();

  build_pick_screen(&scr_red, &lbl_red_pick,
                    "SELECT RED", lv_color_make(255, 60, 60),
                    lv_color_make(255, 180, 180),
                    lv_color_make(255, 120, 120),
                    lv_color_make(255, 200, 220),
                    lv_color_make(255, 150, 200),
                    on_pick_right, on_pick_left, on_pick_kevin_match, on_pick_kevin_skills);

  build_pick_screen(&scr_blue, &lbl_blue_pick,
                    "SELECT BLUE", lv_color_make(80, 160, 255),
                    lv_color_make(180, 220, 255),
                    lv_color_make(140, 200, 255),
                    lv_color_make(200, 210, 255),
                    lv_color_make(160, 190, 255),
                    on_pick_right_blue, on_pick_left_blue, on_pick_kevin_match_blue, on_pick_kevin_skills_blue);

  build_skills();
  build_debug();
}

// ---------------------------
// LVGL task
// ---------------------------
static void lvgl_task_fn(void*) {
  build_all();
  load(ScreenId::Main);

  while (true) {
    lv_timer_handler();
    pros::delay(5);
  }
}

// ---------------------------
// Public entry
// ---------------------------
void Start() {
  static bool started = false;
  if (started) return;
  started = true;

  static pros::Task t(lvgl_task_fn, nullptr, "LVGL GUI");
}

}  // namespace gui
}  // namespace aon
