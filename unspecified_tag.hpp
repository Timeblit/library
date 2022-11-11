#pragma once

struct unspecified_tag
{
   unspecified_tag            ()                       = delete;
   unspecified_tag            (const unspecified_tag&) = delete;
   unspecified_tag            (unspecified_tag&&)      = delete;
   unspecified_tag& operator= (const unspecified_tag&) = delete;
   unspecified_tag& operator= (unspecified_tag&&)      = delete;
  ~unspecified_tag            ()                       = delete;
};

