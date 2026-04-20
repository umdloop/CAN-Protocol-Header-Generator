#pragma once
#include <string>
#include <cpp-can-parser/CANDatabase.h>

std::string generate(const CppCAN::CANDatabase& db, const std::string& dbc_filename, const std::string& dbc_hash, const std::string& date);
