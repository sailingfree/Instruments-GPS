// Display information on the screen

#include <display.h>
#include <cyd_pins.h>
#include <lvgl.h>
#include <esp32_smartdisplay.h>
#include <myFonts.h>
#include <StringStream.h>
#include <N2kMessages.h>
#include <NMEA0183Messages.h>
#include <N2kMsg.h>
#include <map>

static const uint32_t border = 1, padding = 0;

static void buttonHandler(lv_event_t* e);

// GNSS Signal strength
static lv_chart_series_t* GNSSChartSeries;
static lv_obj_t* GNSSChart;

// GNSSS sky view
static lv_obj_t* skyView;

// Static local cache of satellite data
#define MAXSATS 9

struct SatData {
    lv_obj_t* dot;
};

static SatData satData[MAXSATS];

lv_obj_t* screens[SCR_MAX];
static Indicator* ind[SCR_MAX][12];
static InfoBar* bars[SCR_MAX];
// define text areas
static lv_obj_t* textAreas[SCR_MAX];

// Constructor. Binds to the parent object.
Indicator::Indicator(lv_obj_t* parent, const char* name, uint32_t x, uint32_t y) {
    container = lv_obj_create(parent);
    lv_obj_set_pos(container, x, y);
    lv_obj_set_width(container, IND_WIDTH - (2 * padding));
    lv_obj_set_height(container, IND_HEIGHT - (2 * padding));

    lv_style_init(&style);
    lv_style_set_border_width(&style, border);
    lv_obj_add_style(container, &style, 0);

    lv_obj_set_layout(container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);

    label = lv_label_create(container);
    lv_label_set_text(label, name);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_style_init(&text_style);
    lv_style_set_text_font(&text_style, &RobotoCondensedVariableFont_wght8);
    lv_obj_add_style(label, &text_style, 0);

    text = lv_label_create(container);
    lv_style_init(&value_style);
    lv_style_set_text_font(&value_style, &RobotoCondensedVariableFont_wght32);
    lv_obj_add_style(text, &value_style, 0);
    lv_obj_set_style_text_align(text, LV_TEXT_ALIGN_CENTER, 0);

    lv_label_set_text(text, "---");
}

// Change the text size
void Indicator::setFont(const lv_font_t* value) {
    lv_style_set_text_font(&value_style, value);
}

// set the value using a double and precision.
void Indicator::setValue(double value, const char* units, uint32_t prec) {
    String v(value, prec);
    v += units;
    setValue(v.c_str());
}

void Indicator::setValue(const char* value) {
    lv_label_set_text(text, value);
}

// Constructor. Binds to the parent object.
// Info bar has the screen title and the time
InfoBar::InfoBar(lv_obj_t* parent, uint32_t y) {
    static lv_style_t style;
    static lv_style_t value_style;

    container = lv_obj_create(parent);
    lv_obj_set_pos(container, 0, y);
    lv_obj_set_width(container, (BAR_WIDTH) - (2 * padding));
    lv_obj_set_height(container, (BAR_HEIGHT)-2 * padding);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);

    lv_style_init(&style);
    lv_style_set_border_width(&style, border);

    lv_style_set_radius(&style, 3);

    lv_style_set_bg_opa(&style, LV_OPA_100);
    lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_BLUE));

    lv_obj_add_style(container, &style, 0);

    //    lv_obj_set_layout(container, LV_LAYOUT_FLEX);
    //    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);

    // Title text
    text = lv_label_create(container);
    lv_style_init(&value_style);
    lv_style_set_bg_opa(&value_style, LV_OPA_100);
    lv_style_set_bg_color(&value_style, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_text_color(&value_style, lv_color_white());
    lv_style_set_pad_all(&value_style, 10);
    lv_style_set_text_font(&value_style, &RobotoCondensedVariableFont_wght24);
    lv_obj_add_style(text, &value_style, 0);
    lv_obj_set_align(text, LV_ALIGN_LEFT_MID);

    curTime = lv_label_create(container);
    lv_obj_add_style(curTime, &value_style, 0);
    lv_label_set_text(curTime, "00:00:00");
    lv_obj_set_align(curTime, LV_ALIGN_RIGHT_MID);
}

MenuBar::MenuBar(lv_obj_t* parent, uint32_t y) {
    // Constructor. Binds to the parent object.
    static lv_style_t style;

    container = lv_obj_create(parent);
    lv_obj_set_pos(container, 0, y);
    lv_obj_set_width(container, (BAR_WIDTH)-2 * padding);
    lv_obj_set_height(container, BAR_MENU_HEIGHT);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);

    lv_style_init(&style);
    lv_style_set_border_width(&style, border);
    lv_style_set_pad_all(&style, 0);
    lv_obj_add_style(container, &style, 0);

    lv_obj_set_layout(container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);
}

static void buttonHandler(lv_event_t* e) {
    void* target = lv_event_get_user_data(e);
    Screens s = reinterpret_cast<Screens&>(target);
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSED) {
        if (s >= 0 && s < SCR_MAX && screens[s]) {
            //  refreshData(s);
            lv_scr_load(screens[s]);
        }
    }
}

// Add a button to a menu bar. The callbackl will change the screen to the target
void MenuBar::addButton(const char* label, Screens target) {
    lv_obj_t* b = lv_button_create(container);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_height(b, BAR_MENU_HEIGHT);
    lv_label_set_text(l, label);
    lv_obj_set_flex_grow(b, 1);
    /*Init the style for the default state*/
    static lv_style_t style;
    lv_style_init(&style);

    lv_style_set_radius(&style, 3);

    lv_style_set_bg_opa(&style, LV_OPA_100);
    lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_BLUE));
    //    lv_style_set_bg_grad_color(&style, lv_palette_darken(LV_PALETTE_BLUE, 2));
    //   lv_style_set_bg_grad_dir(&style, LV_GRAD_);

    lv_style_set_border_opa(&style, LV_OPA_40);
    lv_style_set_border_width(&style, 2);
    lv_style_set_border_color(&style, lv_palette_main(LV_PALETTE_GREY));

    //    lv_style_set_shadow_width(&style, 8);
    //    lv_style_set_shadow_color(&style, lv_palette_main(LV_PALETTE_GREY));
    //    lv_style_set_shadow_offset_y(&style, 8);

    lv_style_set_outline_opa(&style, LV_OPA_COVER);
    lv_style_set_outline_color(&style, lv_palette_main(LV_PALETTE_BLUE));

    lv_style_set_text_color(&style, lv_color_white());
    lv_style_set_text_font(&style, &RobotoCondensedVariableFont_wght32);
    //    lv_style_set_pad_all(&style, 10);
    //    lv_obj_remove_style_all(b);
    lv_obj_add_style(b, &style, 0);
    lv_event_code_t code = LV_EVENT_PRESSED;
    //,             /**< The object has been pressed*/
    // LV_EVENT_PRESSING,            /**< The object is being pressed (called continuously while pressing)*/
    // LV_EVENT_PRESS_LOST,          /**< The object is still being pressed but slid cursor/finger off of the object */
    // LV_EVENT_SHORT_CLICKED,       /**< The object was pressed for a short period of time, then released it. Not called if scrolled.*/
    // LV_EVENT_LONG_PRESSED,        /**< Object has been pressed for at least `long_press_time`.  Not called if scrolled.*/
    // LV_EVENT_LONG_PRESSED_REPEAT, /**< Called after `long_press_time` in every `long_press_repeat_time` ms.  Not called if scrolled.*/
    // LV_EVENT_CLICKED,             /**< Called on release if not scrolled (regardless to long press)*/
    // LV_EVENT_RELEASED,
    lv_obj_add_event_cb(b, buttonHandler, code, (void*)target);
}

// Add a button to a menu bar. The callback will change the screen to the target
// returns a pointer to the label object
lv_obj_t* MenuBar::addActionButton(const char* label, void (*ptr)(lv_event_t* e)) {
    lv_obj_t* b = lv_button_create(container);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, label);
    lv_obj_set_flex_grow(b, 1);
    /*Init the style for the default state*/
    static lv_style_t style;
    lv_style_init(&style);

    lv_style_set_radius(&style, 3);

    lv_style_set_bg_opa(&style, LV_OPA_100);
    lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_color(&style, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_dir(&style, LV_GRAD_DIR_VER);

    lv_style_set_border_opa(&style, LV_OPA_40);
    lv_style_set_border_width(&style, 2);
    lv_style_set_border_color(&style, lv_palette_main(LV_PALETTE_GREY));

    lv_style_set_shadow_width(&style, 8);
    lv_style_set_shadow_color(&style, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_shadow_offset_y(&style, 8);

    lv_style_set_outline_opa(&style, LV_OPA_COVER);
    lv_style_set_outline_color(&style, lv_palette_main(LV_PALETTE_BLUE));

    lv_style_set_text_color(&style, lv_color_white());
    lv_style_set_pad_all(&style, 10);
    //    lv_obj_remove_style_all(b);
    lv_obj_add_style(b, &style, 0);
    lv_obj_add_event_cb(b, ptr, LV_EVENT_CLICKED, NULL);

    return l;
}

void InfoBar::setValue(const char* value) {
    lv_label_set_text(text, value);
}

void InfoBar::setTime(const char* t) {
    lv_label_set_text(curTime, t);
}

static void setupCommonstyles(lv_obj_t* obj) {
    static lv_style_t style;
    lv_obj_set_style_pad_gap(obj, padding, 0);

    lv_obj_set_height(obj, BODY_HEIGHT);
    lv_obj_set_width(obj, TFT_WIDTH);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

static void setupHeader(Screens scr, lv_obj_t* screen, const char* title) {
    // Info bar at the tope
    InfoBar* bar = new InfoBar(screen, BAR_ROW_TOP);
    bars[scr] = bar;
    bar->setValue(title);
}

static void setupMenu(lv_obj_t* screen) {
    MenuBar* menuBar = new MenuBar(screen, BAR_ROW_BOTTOM);
    menuBar->addButton("GPS", SCR_GPS);
    menuBar->addButton("Sky", SCR_SKY);
    menuBar->addButton("Info", SCR_INFO1);
}

lv_obj_t* createGpsScreen() {
    lv_obj_t* screen = lv_obj_create(NULL);

    setupCommonstyles(screen);
    setupHeader(SCR_GPS, screen, "GPS");

    ind[SCR_GPS][GNSS_HDOP] = new Indicator(screen, "HDOP", COL1, ROW1);
    ind[SCR_GPS][GNSS_SATS] = new Indicator(screen, "Sats", COL2, ROW1);
    ind[SCR_GPS][GNSS_LAT] = new Indicator(screen, "LAT", COL1, ROW2);
    ind[SCR_GPS][GNSS_LONG] = new Indicator(screen, "LON", COL2, ROW2);
    ind[SCR_GPS][GNSS_SOG] = new Indicator(screen, "SOG Kts", COL1, ROW3);
    ind[SCR_GPS][GNSS_COG] = new Indicator(screen, "COG deg", COL2, ROW3);

    setupMenu(screen);
    return screen;
}

// Define the expected min and max values for the GNSS SNR
#define MIN_SNR 35
#define MAX_SNR 50
lv_obj_t* createSkyScreen() {
    lv_obj_t* screen = lv_obj_create(NULL);
    setupCommonstyles(screen);
    setupHeader(SCR_SKY, screen, "GPS Sky");

    // Create a sky view. An image forms the background rings
    LV_IMG_DECLARE(sky);
    skyView = lv_image_create(screen);
    lv_img_set_src(skyView, &sky);
    lv_obj_set_pos(skyView, 0, BAR_HEIGHT);
    lv_obj_set_width(skyView, TFT_WIDTH / 2);
    lv_obj_set_height(skyView, BODY_HEIGHT);

    // Chart for the signal strength
    GNSSChart = lv_chart_create(screen);
    lv_obj_set_pos(GNSSChart, TFT_WIDTH / 2, BAR_HEIGHT);
    lv_obj_set_width(GNSSChart, TFT_WIDTH / 2);
    lv_obj_set_height(GNSSChart, BODY_HEIGHT);

    lv_chart_set_type(GNSSChart, LV_CHART_TYPE_BAR);
    lv_chart_set_range(GNSSChart, LV_CHART_AXIS_PRIMARY_Y, MIN_SNR, MAX_SNR);  // Typical min and max SNR
                                                                               //    lv_chart_set_range(GNSSChart, LV_CHART_AXIS_PRIMARY_X, 1, MAXSATS);

    GNSSChartSeries = lv_chart_add_series(GNSSChart, lv_palette_lighten(LV_PALETTE_GREEN, 2), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_point_count(GNSSChart, MAXSATS);

    setupMenu(screen);
    return screen;
}

lv_obj_t* createInfo1Screen() {
    lv_obj_t* screen = lv_obj_create(NULL);
    setupCommonstyles(screen);
    setupHeader(SCR_INFO1, screen, "System");

    setupMenu(screen);
    return screen;
}

void setup_display() {
    smartdisplay_init();
    smartdisplay_lcd_set_backlight(1.0f);
    lv_display_set_rotation(NULL, LV_DISPLAY_ROTATION_90);

    lv_theme_t* theme = NULL;

    lv_disp_t* dispp = lv_disp_get_default();
    theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                  false, &RobotoCondensedVariableFont_wght24);

    // theme = lv_theme_mono_init(dispp, false, &lv_font_montserrat_24);
    theme = lv_theme_mono_init(dispp, false, &RobotoCondensedVariableFont_wght32);

    if (theme) {
        lv_disp_set_theme(dispp, theme);
    }
    // Create the screens
    screens[SCR_GPS] = createGpsScreen();
    screens[SCR_SKY] = createSkyScreen();
    screens[SCR_INFO1] = createInfo1Screen();

    lv_scr_load(screens[SCR_GPS]);
}

void display_write(MeterIdx obj, double value, const char* units, uint32_t prec) {
    ind[SCR_GPS][obj]->setValue(value, units, prec);
}

// Upadte the time on the screen
void updateTime(StringStream t) {
    bars[SCR_GPS]->setTime(t.data.c_str());
    bars[SCR_SKY]->setTime(t.data.c_str());
    bars[SCR_INFO1]->setTime(t.data.c_str());
}

// Update the meters. Called regularly from the main loop/task
void metersWork(void) {
    static const uint32_t tick_delay = 50;
    lv_task_handler(); /* let the GUI do its work */
    lv_tick_inc(tick_delay);
    delay(tick_delay);
}

// set a value in the GNSSChart
// The value should be mapped to be between the max and min range
void setGNSSSignal(uint32_t idx, uint32_t val) {
    if (idx < 0 || idx > MAXSATS)
        return;  // Ignore bad index
    lv_chart_set_value_by_id(GNSSChart, GNSSChartSeries, idx, val);
}

// set one of the indicators in the sjky view
void setGNSSSky(uint32_t idx, double azimuth, double declination) {
    if (idx < 0 || idx > MAXSATS)
        return;  // Ignore bad index
    if (azimuth < 0 || azimuth > 360 || declination < 0 || declination > 90) {
        return;  // Ignore inplausible values
    }

    // Green dot for the sky view
    if (!satData[idx].dot) {
        // First time for this index so create the image object
        LV_IMG_DECLARE(green_dot);
        satData[idx].dot = lv_img_create(skyView);
        lv_img_set_src(satData[idx].dot, &green_dot);
    }
    uint32_t dotw, doth;
    dotw = lv_obj_get_width(satData[idx].dot);
    doth = lv_obj_get_height(satData[idx].dot);
    uint32_t skyw, skyh;
    skyw = lv_obj_get_width(skyView);
    skyh = lv_obj_get_height(skyView);
    double rad = skyw / 2 - dotw;
    rad *= cos(DegToRad(declination));
    int32_t x = sin(DegToRad(azimuth)) * rad;
    int32_t y = cos(DegToRad(azimuth)) * rad;
    int32_t xorig = skyw / 2 - dotw / 2;
    int32_t yorig = skyh / 2 - doth / 2;
    lv_obj_set_pos(satData[idx].dot, xorig + x, yorig - y);
}

// Initialise the sky view for the nunber of satellites. Removes any old ones not needed
void initGNSSSky(uint32_t svs) {
    for (int i = svs; i < MAXSATS; i++) {
        if (satData[i].dot) {
            lv_obj_del(satData[i].dot);
            satData[i].dot = NULL;
        }
    }
}

// Init the signal display to the number of SVs
void initGNSSSignal(uint32_t svs) {
    lv_chart_set_all_value(GNSSChart, GNSSChartSeries, 0);
}

// Map for the satellite informations
extern std::map<int, tGSV> Satellites;

void updateGnss() {
    std::map<int, tGSV>::iterator it = Satellites.begin();
    uint32_t idx = 0;  // Index into sky view
    initGNSSSky(0);
    initGNSSSignal(0);
    while (it != Satellites.end()) {
        tGSV sat = it->second;
        if (sat.Azimuth != NMEA0183DoubleNA && sat.Elevation != NMEA0183DoubleNA && sat.SNR != NMEA0183DoubleNA) {
            setGNSSSky(idx, sat.Azimuth, sat.Elevation);
            // Map the value to reasonalbe range
            uint32_t snr = map(sat.SNR, 1, 99, MIN_SNR, MAX_SNR);
            setGNSSSignal(idx, snr);
            idx++;
        }
        it++;
        lv_chart_refresh(GNSSChart);
    }
}