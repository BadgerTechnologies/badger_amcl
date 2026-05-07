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

#ifndef AMCL_LOG_HPP
#define AMCL_LOG_HPP

#include <cstdarg>
#include <functional>
#include <string>

namespace amcl
{

// Logging interface for core algorithm code (pf/, map/, sensors/).
//
// The node calls setLogger() at startup with lambdas that forward to the host
// logging API. Core code uses getLogger() only; changing backends means
// changing what setLogger() receives, not editing algorithm sources.
//
// Until setLogger() runs, amcl_log.cpp prints to stdout/stderr so tests can run
// without wiring a node-provided logger.
struct Logger
{
  std::function<void(const std::string&)> info = [](const std::string&) {};
  std::function<void(const std::string&)> warn = [](const std::string&) {};
  std::function<void(const std::string&)> error = [](const std::string&) {};
  std::function<void(const std::string&)> debug = [](const std::string&) {};
};

void setLogger(Logger logger);
const Logger& getLogger();

// printf-style helper that returns a std::string, for use with
// getLogger().info(logFormat(...)).
std::string logFormat(const char* fmt, ...) noexcept;

} // namespace amcl

#endif // AMCL_LOG_HPP
