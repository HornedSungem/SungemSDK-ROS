

#ifndef HORNED_SUNGEM_LIB_EXCEPTION_UTIL_H
#define HORNED_SUNGEM_LIB_EXCEPTION_UTIL_H

#include <map>

namespace horned_sungem_lib
{
class ExceptionUtil
{
public:
  static void tryToThrowHsException(int code);
};
}   // namespace horned_sungem_lib
#endif  // HORNED_SUNGEM_LIB_EXCEPTION_UTIL_H

