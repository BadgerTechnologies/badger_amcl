/*
 *  Copyright (C) 2020 Badger Technologies, LLC
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "amcl_log.hpp"

#include <cstdio>
#include <cstring>

namespace amcl
{

namespace
{

Logger g_logger{
    [](const std::string& m) { std::printf("[AMCL INFO]  %s\n", m.c_str()); },
    [](const std::string& m) { std::printf("[AMCL WARN]  %s\n", m.c_str()); },
    [](const std::string& m)
    { std::fprintf(stderr, "[AMCL ERROR] %s\n", m.c_str()); },
    [](const std::string&) {}};

} // namespace

void setLogger(Logger logger)
{
  g_logger = std::move(logger);
}

const Logger& getLogger()
{
  return g_logger;
}

std::string logFormat(const char* fmt, ...) noexcept
{
  std::va_list args;
  va_start(args, fmt);
  std::va_list args_copy;
  va_copy(args_copy, args);
  const int size = std::vsnprintf(nullptr, 0, fmt, args_copy);
  va_end(args_copy);
  va_end(args);
  if (size <= 0)
  {
    return {};
  }
  std::string result(static_cast<std::size_t>(size) + 1, '\0');
  va_start(args, fmt);
  std::vsnprintf(&result[0], result.size(), fmt, args);
  va_end(args);
  result.resize(static_cast<std::size_t>(size));
  return result;
}

} // namespace amcl
