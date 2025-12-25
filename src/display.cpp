#include "display.hpp"
#include "odom.hpp"
#include "riptide.hpp"

namespace display {
    pros::Task *displayTask;

    lv_obj_t *leftMotorsDisplay[3];
    lv_obj_t *rightMotorsDisplay[3];
    lv_obj_t *leftMotorsTempSquares[3];
    lv_obj_t *rightMotorsTempSquares[3];

    lv_obj_t *extraDisplay1;
    lv_obj_t *extraDisplay2;

    void displayMotorCurrentAndTemperature() {
        for (int i = 0; i < 3; i++) {
            int leftCurrent = chass::mleft.get_current_draw_all()[i];
            int rightCurrent = chass::mright.get_current_draw_all()[i];
            int leftTemp = chass::mleft.get_temperature_all()[i];
            int rightTemp = chass::mright.get_temperature_all()[i];

            lv_label_set_text_fmt(leftMotorsDisplay[i], "L%d: %.1fA", i + 1, leftCurrent / 1000.0);
            lv_label_set_text_fmt(rightMotorsDisplay[i], "R%d: %.1fA", i + 1, rightCurrent / 1000.0);

            if (leftTemp >= 60) {
                lv_obj_set_style_bg_color(leftMotorsTempSquares[i], lv_color_hex(0xFF0000), 0);
            } else {
                lv_obj_set_style_bg_color(leftMotorsTempSquares[i], lv_color_hex(0x00FF00), 0);
            }

            if (rightTemp >= 60) {
                lv_obj_set_style_bg_color(rightMotorsTempSquares[i], lv_color_hex(0xFF0000), 0);
            } else {
                lv_obj_set_style_bg_color(rightMotorsTempSquares[i], lv_color_hex(0x00FF00), 0);
            }
        }
    }

    void update() {
        // displayMotorCurrentAndTemperature();

        lv_label_set_text_fmt(extraDisplay1, "colorSortSensor: %d", (int)(intk::colorSortSensor.get_hue() * 100));
        lv_label_set_text_fmt(extraDisplay2, "Angle: %d deg", (int)(odom::getHeading() * 100));
    }

    void initialize() {
        // leftMotorsDisplay[0] = lv_label_create(lv_scr_act());
        // lv_obj_align(leftMotorsDisplay[0], LV_ALIGN_TOP_LEFT, 10, 10);

        // rightMotorsDisplay[0] = lv_label_create(lv_scr_act());
        // lv_obj_align(rightMotorsDisplay[0], LV_ALIGN_TOP_LEFT, 10, 40);

        extraDisplay1 = lv_label_create(lv_screen_active());
        lv_obj_align(extraDisplay1, LV_ALIGN_TOP_LEFT, 10, 80);

        extraDisplay2 = lv_label_create(lv_screen_active());
        lv_obj_align(extraDisplay2, LV_ALIGN_TOP_LEFT, 10, 120);

        displayTask = new pros::Task([]() {
            const int loopDelay = 100;
            while (true) {
                update();
                pros::delay(loopDelay);
            }
        });
    }
} // namespace display