// ProffieOS/props/color_resolver.h
#ifndef PROPS_COLOR_RESOLVER_H
#define PROPS_COLOR_RESOLVER_H

#include "../common/color.h"

struct NamedColorEntry {
  const char* name;
  uint8_t r, g, b;
};

const NamedColorEntry named_color_table[] = {
  {"aliceblue", 240, 248, 255},
  {"aqua", 0, 255, 255},
  {"aquamarine", 127, 255, 212},
  {"azure", 240, 255, 255},
  {"bisque", 255, 228, 196},
  {"black", 0, 0, 0},
  {"blue", 0, 0, 255},
  {"blueviolet", 138, 43, 226},
  {"chartreuse", 127, 255, 0},
  {"coral", 255, 127, 80},
  {"cornsilk", 255, 248, 220},
  {"cyan", 0, 255, 255},
  {"darkmagenta", 139, 0, 139},
  {"darkorange", 255, 140, 0},
  {"deeppink", 255, 20, 147},
  {"deepskyblue", 0, 191, 255},
  {"dodgerblue", 30, 144, 255},
  {"floralwhite", 255, 250, 240},
  {"ghostwhite", 248, 248, 255},
  {"gold", 255, 215, 0},
  {"green", 0, 128, 0},
  {"greenyellow", 173, 255, 47},
  {"honeydew", 240, 255, 240},
  {"hotpink", 255, 105, 180},
  {"indianred", 205, 92, 92},
  {"ivory", 255, 255, 240},
  {"lavender", 230, 230, 250},
  {"lavenderblush", 255, 240, 245},
  {"lawngreen", 124, 252, 0},
  {"lemonchiffon", 255, 250, 205},
  {"lightcyan", 224, 255, 255},
  {"lightskyblue", 135, 206, 250},
  {"lime", 0, 255, 0},
  {"limegreen", 50, 205, 50},
  {"magenta", 255, 0, 255},
  {"maroon", 128, 0, 0},
  {"mediumspringgreen", 0, 250, 154},
  {"mintcream", 245, 255, 250},
  {"mistyrose", 255, 228, 225},
  {"moccasin", 255, 228, 181},
  {"navy", 0, 0, 128},
  {"olive", 128, 128, 0},
  {"orange", 255, 165, 0},
  {"orangered", 255, 69, 0},
  {"palegreen", 152, 251, 152},
  {"peachpuff", 255, 218, 185},
  {"pink", 255, 192, 203},
  {"plum", 221, 160, 221},
  {"purple", 128, 0, 128},
  {"red", 255, 0, 0},
  {"royalblue", 65, 105, 225},
  {"salmon", 250, 128, 114},
  {"seagreen", 46, 139, 87},
  {"sienna", 160, 82, 45},
  {"silver", 192, 192, 192},
  {"skyblue", 135, 206, 235},
  {"snow", 255, 250, 250},
  {"springgreen", 0, 255, 127},
  {"steelblue", 70, 130, 180},
  {"teal", 0, 128, 128},
  {"tomato", 255, 99, 71},
  {"turquoise", 64, 224, 208},
  {"violet", 238, 130, 238},
  {"white", 255, 255, 255},
  {"yellow", 255, 255, 0},
  {"yellowgreen", 154, 205, 50},
};

const int NAMED_COLOR_COUNT = sizeof(named_color_table) / sizeof(named_color_table[0]);

bool ResolveColor(const char* str, uint16_t* r16, uint16_t* g16, uint16_t* b16) {
  if (!str || !str[0]) return false;

  const char* p = str;
  if (*p == '(') p++;

  char* end;
  long r = strtol(p, &end, 10);
  if (end != p && *end == ',') {
    p = end + 1;
    long g = strtol(p, &end, 10);
    if (end != p && *end == ',') {
      p = end + 1;
      long b = strtol(p, &end, 10);
      if (end != p && (*end == ')' || *end == 0 || *end == ' ')) {
        *r16 = (uint16_t)(constrain(r, 0, 255)) * 257;
        *g16 = (uint16_t)(constrain(g, 0, 255)) * 257;
        *b16 = (uint16_t)(constrain(b, 0, 255)) * 257;
        return true;
      }
    }
  }

  for (int i = 0; i < NAMED_COLOR_COUNT; i++) {
    if (strcasecmp(str, named_color_table[i].name) == 0) {
      *r16 = (uint16_t)named_color_table[i].r * 257;
      *g16 = (uint16_t)named_color_table[i].g * 257;
      *b16 = (uint16_t)named_color_table[i].b * 257;
      return true;
    }
  }

  return false;
}

int ColorToStyleArg(const char* color_str, char* buf, int buf_size) {
  uint16_t r, g, b;
  if (!ResolveColor(color_str, &r, &g, &b)) {
    r = g = b = 65535;
  }
  return snprintf(buf, buf_size, "%u,%u,%u", r, g, b);
}

#endif // PROPS_COLOR_RESOLVER_H
