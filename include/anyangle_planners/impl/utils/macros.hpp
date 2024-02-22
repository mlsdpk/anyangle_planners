#pragma once

#include <memory>

#define ANYANGLE_OBJ_FORWARD(C)      \
  class C;                           \
  using C##Ptr = std::shared_ptr<C>; \
  using C##ConstPtr = std::shared_ptr<const C>;
