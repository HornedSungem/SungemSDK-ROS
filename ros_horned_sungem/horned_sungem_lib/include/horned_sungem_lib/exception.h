

#ifndef HORNED_SUNGEM_LIB_EXCEPTION_H
#define HORNED_SUNGEM_LIB_EXCEPTION_H

#include <exception>
#include <string>

namespace horned_sungem_lib
{
struct HSException: public std::exception {};

struct HsException: public HSException
{
  explicit HsException(const std::string& msg)
      : msg_(msg)
  {
  }

  const char* what() const noexcept
  {
    return msg_.c_str();
  }

private:
  const std::string msg_;
};

struct HsBusy: public HsException
{
  explicit HsBusy(const std::string& msg)
    : HsException(msg)
  {
  }
};

struct HsError: public HsException
{
  explicit HsError(const std::string& msg)
    : HsException(msg)
  {
  }
};

struct HsOutOfMemory: public HsException
{
  explicit HsOutOfMemory(const std::string& msg)
    : HsException(msg)
  {
  }
};

struct HsDeviceNotFound: public HsException
{
  explicit HsDeviceNotFound(const std::string& msg)
    : HsException(msg)
  {
  }
};

struct HsInvalidParameters: public HsException
{
  explicit HsInvalidParameters(const std::string& msg)
    : HsException(msg)
  {
  }
};

struct HsTimeout: public HsException
{
  explicit HsTimeout(const std::string& msg)
    : HsException(msg)
  {
  }
};

struct HsMvCmdNotFound: public HsException
{
  explicit HsMvCmdNotFound(const std::string& msg)
    : HsException(msg)
  {
  }
};

struct HsNoData: public HsException
{
  explicit HsNoData(const std::string& msg)
    : HsException(msg)
  {
  }
};

struct HsGone: public HsException
{
  explicit HsGone(const std::string& msg)
    : HsException(msg)
  {
  }
};

struct HsUnsupportedGraphFile: public HsException
{
  explicit HsUnsupportedGraphFile(const std::string& msg)
    : HsException(msg)
  {
  }
};

struct HsMyriadError: public HsException
{
  explicit HsMyriadError(const std::string& msg)
    : HsException(msg)
  {
  }
};

struct HsThermalException: public HSException
{
  virtual const char* what() const noexcept = 0;
};

struct HsHighThermal: public HsThermalException
{
  const char* what() const noexcept;
};

struct HsAggressiveThermal: public HsThermalException
{
  const char* what() const noexcept;
};

struct HSGraphException: public HSException
{
  virtual const char* what() const noexcept = 0;
};

struct HSGraphFileError: public HSGraphException
{
  const char* what() const noexcept;
};

struct HSMeanAndStddevError: public HSGraphException
{
  const char* what() const noexcept;
};

struct HSInputSizeFileError: public HSGraphException
{
  const char* what() const noexcept;
};

struct HSInputSizeError: public HSGraphException
{
  const char* what() const noexcept;
};

struct HSLoadCategoriesError: public HSGraphException
{
  const char* what() const noexcept;
};
}  // namespace horned_sungem_lib
#endif  // HORNED_SUNGEM_LIB_EXCEPTION_H
