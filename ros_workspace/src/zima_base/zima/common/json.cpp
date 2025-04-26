/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/json.h"

#include "zima/logger/logger.h"

namespace zima {

bool JsonHelper::TryParse(const std::string& str, Json& json) {
  try {
    json = Json::parse(str);
    return true;
  } catch (Json::parse_error& error) {
    ZERROR << error.what();
    ZERROR << "Input string:\n" << str;
  };
  return false;
}

bool JsonHelper::KeyExists(const Json& json, const std::string& key) {
  return json.count(key) > 0;
}

bool JsonHelper::GetBoolean(const Json& json, const std::string& key,
                            bool& value) {
  if (KeyExists(json, key) && json[key].is_boolean()) {
    json[key].get_to(value);
    return true;
  }
  return false;
}

bool JsonHelper::GetInt(const Json& json, const std::string& key, int& value) {
  if (KeyExists(json, key) && json[key].is_number_integer()) {
    json[key].get_to(value);
    return true;
  }
  return false;
}

bool JsonHelper::GetUInt(const Json& json, const std::string& key,
                         uint& value) {
  if (KeyExists(json, key) && json[key].is_number_unsigned()) {
    json[key].get_to(value);
    return true;
  }
  return false;
}

bool JsonHelper::GetFloat(const Json& json, const std::string& key,
                          float& value) {
  if (KeyExists(json, key) && json[key].is_number()) {
    json[key].get_to(value);
    return true;
  }
  return false;
}

bool JsonHelper::GetString(const Json& json, const std::string& key,
                           std::string& value) {
  if (KeyExists(json, key) && json[key].is_string()) {
    json[key].get_to(value);
    return true;
  }
  return false;
}

bool JsonHelper::GetObject(const Json& json, const std::string& key,
                           Json& value) {
  if (KeyExists(json, key) && json[key].is_object()) {
    json[key].get_to(value);
    return true;
  }
  return false;
}

bool JsonHelper::GetArray(const Json& json, const std::string& key,
                          Json& value) {
  if (KeyExists(json, key) && json[key].is_array()) {
    json[key].get_to(value);
    return true;
  }
  return false;
}

}  // namespace zima
