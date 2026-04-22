// Copyright 2026 KC-ML2
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace text_nav_bridge
{
namespace text_utils
{

inline double levenshteinDistance(const std::string & s1, const std::string & s2)
{
  const size_t m = s1.size();
  const size_t n = s2.size();

  if (m == 0) {return n;}
  if (n == 0) {return m;}

  size_t max_len = std::max(m, n);
  size_t len_diff = std::abs(static_cast<int>(m) - static_cast<int>(n));

  if (len_diff > max_len * 0.6) {
    return len_diff;
  }

  double max_allowed_dist = max_len * 0.5;
  if (len_diff > max_allowed_dist) {
    return len_diff;
  }

  std::vector<int> prev_row(n + 1);
  std::vector<int> curr_row(n + 1);

  for (size_t j = 0; j <= n; j++) {
    prev_row[j] = j;
  }

  for (size_t i = 1; i <= m; i++) {
    curr_row[0] = i;
    int row_min = curr_row[0];

    for (size_t j = 1; j <= n; j++) {
      int cost = (std::tolower(static_cast<unsigned char>(s1[i - 1])) ==
        std::tolower(static_cast<unsigned char>(s2[j - 1]))) ? 0 : 1;
      curr_row[j] = std::min(
        {
          prev_row[j] + 1,
          curr_row[j - 1] + 1,
          prev_row[j - 1] + cost
        });
      row_min = std::min(row_min, curr_row[j]);
    }

    if (row_min > static_cast<int>(max_allowed_dist)) {
      return max_len;
    }

    std::swap(prev_row, curr_row);
  }

  return prev_row[n];
}

inline double textSimilarity(const std::string & s1, const std::string & s2)
{
  if (s1.empty() || s2.empty()) {return 0.0;}

  double dist = levenshteinDistance(s1, s2);
  double max_len = std::max(s1.size(), s2.size());
  double lev_sim = 1.0 - (dist / max_len);

  const std::string & shorter_ref = (s1.size() < s2.size()) ? s1 : s2;
  const std::string & longer_ref = (s1.size() >= s2.size()) ? s1 : s2;

  auto ci_find = [](const std::string & haystack, const std::string & needle) {
      auto it = std::search(
        haystack.begin(), haystack.end(),
        needle.begin(), needle.end(),
        [](unsigned char a, unsigned char b) {
          return std::tolower(a) == std::tolower(b);
        });
      return it != haystack.end();
    };

  // Substring bonus: boost similarity when shorter text is contained in longer text
  // e.g. "B1" vs "B1 Floor", "Exit" vs "Exit Sign"
  if (ci_find(longer_ref, shorter_ref)) {
    double coverage = static_cast<double>(shorter_ref.size()) / longer_ref.size();
    double substring_bonus = 0.3 * coverage;
    lev_sim = std::min(1.0, lev_sim + substring_bonus);
  }

  // Prefix bonus: boost similarity when the first 3-4 characters match
  // Helps handle OCR typos in later characters (e.g. "restroom" vs "restrom")
  size_t prefix_len = std::min(shorter_ref.size(), static_cast<size_t>(4));
  if (prefix_len >= 3) {
    bool prefix_match = true;
    for (size_t i = 0; i < prefix_len; ++i) {
      if (std::tolower(static_cast<unsigned char>(shorter_ref[i])) !=
        std::tolower(static_cast<unsigned char>(longer_ref[i])))
      {
        prefix_match = false;
        break;
      }
    }

    if (prefix_match) {
      double prefix_bonus = 0.10;
      lev_sim = std::min(1.0, lev_sim + prefix_bonus);
    }
  }

  return lev_sim;
}

}  // namespace text_utils
}  // namespace text_nav_bridge
