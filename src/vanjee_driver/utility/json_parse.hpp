/*********************************************************************************************************************
Copyright (c) 2023 Vanjee
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For Vanjee LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the Vanjee, nor Wanji Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <cctype>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace vanjee {
namespace lidar {

enum class JsonType { Null, Boolean, Number, String, Array, Object };

class JsonValue {
 public:
  virtual ~JsonValue() = default;
  virtual JsonType type() const = 0;
  virtual std::string toString() const = 0;
};

class JsonNull : public JsonValue {
 public:
  JsonType type() const override {
    return JsonType::Null;
  }
  std::string toString() const override {
    return "null";
  }
};

class JsonBoolean : public JsonValue {
  bool value_;

 public:
  JsonBoolean(bool value) : value_(value) {
  }
  JsonType type() const override {
    return JsonType::Boolean;
  }
  bool value() const {
    return value_;
  }
  std::string toString() const override {
    return value_ ? "true" : "false";
  }
};

class JsonNumber : public JsonValue {
  double value_;

 public:
  JsonNumber(double value) : value_(value) {
  }
  JsonType type() const override {
    return JsonType::Number;
  }
  double value() const {
    return value_;
  }
  std::string toString() const override {
    return std::to_string(value_);
  }
};

class JsonString : public JsonValue {
  std::string value_;

 public:
  JsonString(const std::string& value) : value_(value) {
  }
  JsonType type() const override {
    return JsonType::String;
  }
  const std::string& value() const {
    return value_;
  }
  std::string toString() const override {
    return "\"" + value_ + "\"";
  }
};

class JsonArray : public JsonValue {
  std::vector<std::shared_ptr<JsonValue>> values_;

 public:
  JsonType type() const override {
    return JsonType::Array;
  }

  void add(std::shared_ptr<JsonValue> value) {
    values_.push_back(value);
  }

  size_t size() const {
    return values_.size();
  }

  std::shared_ptr<JsonValue> get(size_t index) const {
    if (index >= values_.size()) {
      // throw std::out_of_range("Array index out of range");
      return nullptr;
    }
    return values_[index];
  }

  std::string toString() const override {
    std::string result = "[";
    for (size_t i = 0; i < values_.size(); ++i) {
      if (i > 0)
        result += ", ";
      result += values_[i]->toString();
    }
    result += "]";
    return result;
  }
};

class JsonObject : public JsonValue {
  std::map<std::string, std::shared_ptr<JsonValue>> values_;

 public:
  JsonType type() const override {
    return JsonType::Object;
  }

  void set(const std::string& key, std::shared_ptr<JsonValue> value) {
    values_[key] = value;
  }

  bool has(const std::string& key) const {
    return values_.find(key) != values_.end();
  }

  std::shared_ptr<JsonValue> get(const std::string& key) const {
    auto it = values_.find(key);
    if (it == values_.end()) {
      // throw std::runtime_error("Key not found: " + key);
      return nullptr;
    }
    return it->second;
  }

  std::string toString() const override {
    std::string result = "{";
    bool first = true;
    for (const auto& pair : values_) {
      if (!first)
        result += ", ";
      result += "\"" + pair.first + "\": " + pair.second->toString();
      first = false;
    }
    result += "}";
    return result;
  }
};

class JsonParser {
  std::string input_;
  size_t pos_;

  std::string unescapeString(const std::string& str) {
    std::string result;
    for (size_t i = 0; i < str.size(); ++i) {
      if (str[i] == '\\' && i + 1 < str.size()) {
        switch (str[i + 1]) {
          case 'n':
            result += '\n';
            break;
          case 't':
            result += '\t';
            break;
          case 'r':
            result += '\r';
            break;
          case '\\':
            result += '\\';
            break;
          case '"':
            result += '"';
            break;
          default:
            result += str[i + 1];
            break;
        }
        i++;
      } else {
        result += str[i];
      }
    }
    return result;
  }

  void skipWhitespace() {
    while (pos_ < input_.size() && std::isspace(input_[pos_])) {
      pos_++;
    }
  }

  char currentChar() {
    return pos_ < input_.size() ? input_[pos_] : '\0';
  }

  char nextChar() {
    if (pos_ < input_.size()) {
      return input_[pos_++];
    }
    return '\0';
  }

  void expect(char expected) {
    skipWhitespace();
    char c = nextChar();
    if (c != expected) {
      // throw std::runtime_error(std::string("Expected '") + expected + "' but found '" + c + "'");
      return;
    }
  }

  std::string parseString() {
    expect('"');
    std::string result;
    while (pos_ < input_.size() && input_[pos_] != '"') {
      if (input_[pos_] == '\\') {
        pos_++;
        if (pos_ >= input_.size()) {
          // throw std::runtime_error("Unterminated string");
          return "";
        }
        switch (input_[pos_]) {
          case '"':
            result += '"';
            break;
          case '\\':
            result += '\\';
            break;
          case '/':
            result += '/';
            break;
          case 'b':
            result += '\b';
            break;
          case 'f':
            result += '\f';
            break;
          case 'n':
            result += '\n';
            break;
          case 'r':
            result += '\r';
            break;
          case 't':
            result += '\t';
            break;
          default:
            // throw std::runtime_error("Invalid escape sequence");
            return "";
        }
      } else {
        result += input_[pos_];
      }
      pos_++;
    }
    expect('"');
    return result;
  }

  double parseNumber() {
    size_t start = pos_;
    if (input_[pos_] == '-')
      pos_++;
    while (pos_ < input_.size() && std::isdigit(input_[pos_])) pos_++;
    if (pos_ < input_.size() && input_[pos_] == '.') {
      pos_++;
      while (pos_ < input_.size() && std::isdigit(input_[pos_])) pos_++;
    }
    if (pos_ < input_.size() && (input_[pos_] == 'e' || input_[pos_] == 'E')) {
      pos_++;
      if (pos_ < input_.size() && (input_[pos_] == '+' || input_[pos_] == '-'))
        pos_++;
      while (pos_ < input_.size() && std::isdigit(input_[pos_])) pos_++;
    }
    std::string numStr = input_.substr(start, pos_ - start);
    return std::stod(numStr);
  }

  std::shared_ptr<JsonValue> parseValue() {
    skipWhitespace();
    char c = currentChar();

    if (c == '"') {
      return std::make_shared<JsonString>(parseString());
    } else if (c == '[') {
      return parseArray();
    } else if (c == '{') {
      return parseObject();
    } else if (c == 't' && input_.substr(pos_, 4) == "true") {
      pos_ += 4;
      return std::make_shared<JsonBoolean>(true);
    } else if (c == 'f' && input_.substr(pos_, 5) == "false") {
      pos_ += 5;
      return std::make_shared<JsonBoolean>(false);
    } else if (c == 'n' && input_.substr(pos_, 4) == "null") {
      pos_ += 4;
      return std::make_shared<JsonNull>();
    } else if (c == '-' || std::isdigit(c)) {
      return std::make_shared<JsonNumber>(parseNumber());
    } else {
      // throw std::runtime_error("Unexpected character: " + std::string(1, c));
      return nullptr;
    }
  }

  std::shared_ptr<JsonArray> parseArray() {
    expect('[');
    auto array = std::make_shared<JsonArray>();

    skipWhitespace();
    if (currentChar() == ']') {
      pos_++;
      return array;
    }

    while (true) {
      array->add(parseValue());
      skipWhitespace();
      if (currentChar() == ']') {
        pos_++;
        break;
      }
      expect(',');
    }
    return array;
  }

  std::shared_ptr<JsonObject> parseObject() {
    std::string processed_input = unescapeString(input_);
    input_ = processed_input;
    pos_ = 0;

    expect('{');
    auto obj = std::make_shared<JsonObject>();

    skipWhitespace();
    if (currentChar() == '}') {
      pos_++;
      return obj;
    }

    while (true) {
      std::string key = parseString();
      expect(':');
      auto value = parseValue();
      if (value == nullptr) {
        return nullptr;
      }
      obj->set(key, value);

      skipWhitespace();
      if (currentChar() == '}') {
        pos_++;
        break;
      }
      expect(',');
    }
    return obj;
  }

 public:
  JsonParser(const std::string& input) : input_(input), pos_(0) {
  }

  std::shared_ptr<JsonValue> parse() {
    skipWhitespace();
    auto result = parseValue();
    skipWhitespace();
    if (pos_ < input_.size()) {
      // throw std::runtime_error("Unexpected content after JSON value");
      return nullptr;
    }
    return result;
  }
};

}  // namespace lidar
}  // namespace vanjee
