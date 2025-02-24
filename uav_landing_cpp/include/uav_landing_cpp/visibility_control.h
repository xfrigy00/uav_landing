#ifndef UAV_LANDING_CPP__VISIBILITY_CONTROL_H_
#define UAV_LANDING_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UAV_LANDING_CPP_EXPORT __attribute__ ((dllexport))
    #define UAV_LANDING_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define UAV_LANDING_CPP_EXPORT __declspec(dllexport)
    #define UAV_LANDING_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef UAV_LANDING_CPP_BUILDING_DLL
    #define UAV_LANDING_CPP_PUBLIC UAV_LANDING_CPP_EXPORT
  #else
    #define UAV_LANDING_CPP_PUBLIC UAV_LANDING_CPP_IMPORT
  #endif
  #define UAV_LANDING_CPP_PUBLIC_TYPE UAV_LANDING_CPP_PUBLIC
  #define UAV_LANDING_CPP_LOCAL
#else
  #define UAV_LANDING_CPP_EXPORT __attribute__ ((visibility("default")))
  #define UAV_LANDING_CPP_IMPORT
  #if __GNUC__ >= 4
    #define UAV_LANDING_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define UAV_LANDING_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define UAV_LANDING_CPP_PUBLIC
    #define UAV_LANDING_CPP_LOCAL
  #endif
  #define UAV_LANDING_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // UAV_LANDING_CPP__VISIBILITY_CONTROL_H_
