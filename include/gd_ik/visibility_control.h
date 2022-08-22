#ifndef GD_IK__VISIBILITY_CONTROL_H_
#define GD_IK__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define GD_IK_EXPORT __attribute__((dllexport))
#define GD_IK_IMPORT __attribute__((dllimport))
#else
#define GD_IK_EXPORT __declspec(dllexport)
#define GD_IK_IMPORT __declspec(dllimport)
#endif
#ifdef GD_IK_BUILDING_LIBRARY
#define GD_IK_PUBLIC GD_IK_EXPORT
#else
#define GD_IK_PUBLIC GD_IK_IMPORT
#endif
#define GD_IK_PUBLIC_TYPE GD_IK_PUBLIC
#define GD_IK_LOCAL
#else
#define GD_IK_EXPORT __attribute__((visibility("default")))
#define GD_IK_IMPORT
#if __GNUC__ >= 4
#define GD_IK_PUBLIC __attribute__((visibility("default")))
#define GD_IK_LOCAL __attribute__((visibility("hidden")))
#else
#define GD_IK_PUBLIC
#define GD_IK_LOCAL
#endif
#define GD_IK_PUBLIC_TYPE
#endif

#endif  // GD_IK__VISIBILITY_CONTROL_H_
