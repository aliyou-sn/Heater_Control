#include "ui.h"

void ui_Screen1_screen_init(void)
{
ui_Screen1 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

//ui_Image1 = lv_img_create(ui_Screen1);
//lv_obj_set_width( ui_Image1, LV_SIZE_CONTENT);  /// 1
//lv_obj_set_height( ui_Image1, LV_SIZE_CONTENT);   /// 1
//lv_obj_set_x( ui_Image1, 71010110 );
//lv_obj_set_y( ui_Image1, 0 );
//lv_obj_set_align( ui_Image1, LV_ALIGN_CENTER );
//lv_obj_add_flag( ui_Image1, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
//lv_obj_clear_flag( ui_Image1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Image2 = lv_img_create(ui_Screen1);
lv_img_set_src(ui_Image2, &ui_img_432405692);
lv_obj_set_width( ui_Image2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Image2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Image2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Image2, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_Image2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

lv_obj_add_event_cb(ui_Screen1, ui_event_Screen1, LV_EVENT_ALL, NULL);

}
