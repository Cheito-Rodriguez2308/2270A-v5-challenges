#pragma once
#include <functional>
#include <string>
#include <unordered_map>

template <typename RetT>
class FunctionReader {
 public:
  using Fn = std::function<RetT(void)>;

  void AddFunction(const std::string& key, Fn fn) { map_[key] = std::move(fn); }

  bool Has(const std::string& key) const { return map_.find(key) != map_.end(); }

  RetT Run(const std::string& key) {
    auto it = map_.find(key);
    if (it == map_.end()) return RetT{};
    return it->second();
  }

 private:
  std::unordered_map<std::string, Fn> map_;
};
