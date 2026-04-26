/*

  #include <source_location>
  #include <stdio.h>


void raise_error(std::source_location loc = std::source_location::current())
{
  #ifdef PRINT_ERR
    printf("[ERR]: %s:%d in %s\n", loc.file_name(), static_cast<unsigned int>(loc.line()), loc.function_name());
  #endif
}
*/
