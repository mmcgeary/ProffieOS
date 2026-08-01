// ProffieOS/props/ini_parser.h
#ifndef PROPS_INI_PARSER_H
#define PROPS_INI_PARSER_H

// Section-aware INI file parser built on ProffieOS FileReader.
// Supports [section] headers, key=value pairs, # comments.
// Values are stored as raw strings — callers interpret types.

#include "../common/file_reader.h"

#define INI_MAX_KEY_LEN 32
#define INI_MAX_VALUE_LEN 128
#define INI_MAX_SECTION_LEN 32

enum IniParseResult {
  INI_OK = 0,
  INI_EOF,
  INI_SECTION,
  INI_KEY_VALUE,
  INI_ERROR
};

class IniParser {
public:
  IniParser() : file_open_(false) {}

  bool Open(const char* filename) {
    file_open_ = reader_.Open(filename);
    current_section_[0] = 0;
    return file_open_;
  }

  void Close() {
    reader_.Close();
    file_open_ = false;
  }

  bool IsOpen() const { return file_open_; }

  // Read next meaningful line. Skips blanks and comments.
  IniParseResult Next() {
    if (!file_open_) return INI_EOF;

    while (true) {
      reader_.skipwhite();
      int c = reader_.Peek();

      if (c == -1) return INI_EOF;

      if (c == '#' || c == ';') {
        reader_.skipline();
        continue;
      }

      if (c == '[') {
        reader_.Read();
        int i = 0;
        while (i < INI_MAX_SECTION_LEN - 1) {
          c = reader_.Read();
          if (c == ']' || c == -1 || c == '\n') break;
          current_section_[i++] = c;
        }
        current_section_[i] = 0;
        reader_.skipline();
        return INI_SECTION;
      }

      if (ReadKeyValue()) {
        return INI_KEY_VALUE;
      }

      reader_.skipline();
    }
  }

  const char* Section() const { return current_section_; }
  const char* Key() const { return current_key_; }
  const char* Value() const { return current_value_; }

  bool SeekSection(const char* section_name) {
    while (true) {
      IniParseResult r = Next();
      if (r == INI_EOF) return false;
      if (r == INI_SECTION && strcasecmp(current_section_, section_name) == 0) {
        return true;
      }
    }
  }

  template<typename Handler>
  int ReadSection(Handler handler) {
    int count = 0;
    while (true) {
      IniParseResult r = Next();
      if (r == INI_EOF || r == INI_SECTION) return count;
      if (r == INI_KEY_VALUE) {
        handler(current_key_, current_value_);
        count++;
      }
    }
  }

private:
  bool ReadKeyValue() {
    int i = 0;
    while (i < INI_MAX_KEY_LEN - 1) {
      int c = reader_.Peek();
      if (c == -1 || c == '\n') { reader_.skipline(); return false; }
      if (c == '=' || c == ' ' || c == '\t') break;
      if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '_' || c == '.') {
        current_key_[i++] = c;
        reader_.Read();
      } else {
        reader_.skipline();
        return false;
      }
    }
    if (i == 0) { reader_.skipline(); return false; }
    current_key_[i] = 0;

    reader_.skipspace();
    if (reader_.Peek() != '=') { reader_.skipline(); return false; }
    reader_.Read();
    reader_.skipspace();

    i = 0;
    while (i < INI_MAX_VALUE_LEN - 1) {
      int c = reader_.Read();
      if (c == -1 || c == '\n' || c == '\r') break;
      current_value_[i++] = c;
    }
    while (i > 0 && (current_value_[i-1] == ' ' || current_value_[i-1] == '\t')) {
      i--;
    }
    current_value_[i] = 0;
    return true;
  }

  FileReader reader_;
  bool file_open_;
  char current_section_[INI_MAX_SECTION_LEN];
  char current_key_[INI_MAX_KEY_LEN];
  char current_value_[INI_MAX_VALUE_LEN];
};

#endif // PROPS_INI_PARSER_H
