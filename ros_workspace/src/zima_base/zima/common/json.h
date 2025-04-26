/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_JSON_H
#define ZIMA_JSON_H

#include "zima/common/nlohmann_json.hpp"

namespace zima {

using Json = nlohmann::json;
using JsonSPtr = std::shared_ptr<Json>;

class JsonHelper {
 public:
  JsonHelper() = default;
  ~JsonHelper() = default;

  static bool TryParse(const std::string& str, Json& json);

  static bool KeyExists(const Json& json, const std::string& key);

  static bool GetBoolean(const Json& json, const std::string& key, bool& value);
  static bool GetInt(const Json& json, const std::string& key, int& value);
  static bool GetUInt(const Json& json, const std::string& key, uint& value);
  static bool GetFloat(const Json& json, const std::string& key, float& value);
  static bool GetString(const Json& json, const std::string& key,
                        std::string& value);
  static bool GetObject(const Json& json, const std::string& key, Json& value);
  static bool GetArray(const Json& json, const std::string& key, Json& value);
};

}  // namespace zima

#endif  // ZIMA_JSON_H
