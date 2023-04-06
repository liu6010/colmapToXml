#pragma once
#include <iostream>
using namespace std;

template <typename T>
std::string VectorToCSV(const std::vector<T>& values) {
  std::string string;
  for (const T value : values) {
    string += std::to_string(value) + ", ";
  }
  return string.substr(0, string.length() - 2);
}

bool IsNotWhiteSpace(const int character) {
  return character != ' ' && character != '\n' && character != '\r' &&
         character != '\t';
}

void StringTrim(std::string* str) {
  // StringLeftTrim
  str->erase(str->begin(),
             std::find_if(str->begin(), str->end(), IsNotWhiteSpace));
  // StringRightTrim
  str->erase(std::find_if(str->rbegin(), str->rend(), IsNotWhiteSpace).base(),
             str->end());
}



