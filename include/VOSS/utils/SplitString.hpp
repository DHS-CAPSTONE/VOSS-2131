/**
 * @file split.hpp
 * @author Andrew Hilton (2131N)
 * @brief Function for splitting a string into tokens
 * @version 0.1
 * @date 2025-07-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <sstream>
#include <string>
#include <vector>

namespace voss {
    
/**
 * @brief Splits a String by a Delimiter
 *
 * @param str Input String
 * @param delimiter Delimiter Character
 * @return std::vector<std::string> Vector of string tokens
 */
inline std::vector<std::string> splitStr(const std::string& str, char delimiter)
{
  std::vector<std::string> tokens;  // Tokens
  std::istringstream stream(str);   // Create input stream
  std::string token;                // String Holder

  // While Possible Split by the delimiter until the token is empty
  while (std::getline(stream, token, delimiter))
  {
    if (!token.empty()) { tokens.push_back(token); }
  }

  // Return vector of tokens
  return tokens;
}

} // namespace voss