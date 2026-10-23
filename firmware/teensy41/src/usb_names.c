/* USB product / manufacturer strings. The udev rule (99-jgb-rover-mcu.rules) matches the product
 * name to create /dev/jgb_rover_mcu; the PC driver works without it (falls back to /dev/ttyACM0). */
#include "usb_names.h"

#define PRODUCT_NAME {'j', 'g', 'b', '_', 'r', 'o', 'v', 'e', 'r', '_', 'm', 'c', 'u'}
#define PRODUCT_NAME_LEN 13
#define MANUFACTURER_NAME {'j', 'g', 'b', '_', 'r', 'o', 'v', 'e', 'r'}
#define MANUFACTURER_NAME_LEN 9

struct usb_string_descriptor_struct usb_string_product_name = {
    2 + PRODUCT_NAME_LEN * 2, 3, PRODUCT_NAME};

struct usb_string_descriptor_struct usb_string_manufacturer_name = {
    2 + MANUFACTURER_NAME_LEN * 2, 3, MANUFACTURER_NAME};
