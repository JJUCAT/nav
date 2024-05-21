#ifndef AR_H
#define AR_H

#if defined(WIN32) && !defined(AR_STATIC)
#ifdef AR_BUILD
#define AR_EXPORT __declspec(dllexport)
#else
#define AR_EXPORT __declspec(dllimport)
#endif
#else
#define AR_EXPORT
#endif

/**
 * \brief Main AR namespace.
 */
namespace dock_visual_perception
{
/**
 * \brief Major version number.
 */
static const int AR_VERSION_MAJOR = 1;

/**
 * \brief Minor version number.
 */
static const int AR_VERSION_MINOR = 0;

/**
 * \brief Patch version number.
 */
static const int AR_VERSION_PATCH = 0;

/**
 * \brief Tag version string.
 *
 * The tag contains alpha, beta and release candidate versions.
 */
static const char* AR_VERSION_TAG = "";

/**
 * \brief Revision version string.
 *
 * The revision contains an identifier from the source control system.
 */
static const char* AR_VERSION_REVISION = "1";

/**
 * \brief Entire version string.
 */
static const char* AR_VERSION = "1.0";

/**
 * \brief Entire version string without dots.
 */
static const char* AR_VERSION_NODOTS = "200";

/**
 * \brief Date the library was built.
 */
static const char* AR_DATE = "2022-08-20";

/**
 * \brief System the library was built on.
 */
static const char* AR_SYSTEM = "Linux 5.15.0-24-generic x86_64";

}

#endif
