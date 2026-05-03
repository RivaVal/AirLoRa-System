
//========  ФАЙЛ ::utils.cpp

#include "utils.h"

void raise_error(std::source_location loc = std::source_location::current())
{
  #ifdef PRINT_ERR
    printf("[ERR]: %s:%d in %s\n", loc.file_name(), static_cast<unsigned int>(loc.line()), loc.function_name());
  #endif
}
//
//===================
//пример использования
//===================
/*

bool some_func(const char * data_ptr)
{
  if(data_ptr == nullptr) { raise_error(); return false; }
  return true;
}

*/