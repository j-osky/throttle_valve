/* Copyright 2010-2022 The MathWorks, Inc. */

#ifndef slsv_diagnostic_codegen_c_api_h
#define slsv_diagnostic_codegen_c_api_h

#include "tmwtypes.h"

#ifdef BUILDING_LIBMWSL_SERVICES
/* being included from inside sl_services module */
#include "package.h"
#define SL_SERVICES_PUBLISHED_C extern "C" DLL_EXPORT_SYM
#else
/* being included from outside, such as raccel code */
#if defined(__cplusplus)
/* Needed for non-inlined C++ S-functions */
#define SL_SERVICES_PUBLISHED_C extern "C"
#else
#define SL_SERVICES_PUBLISHED_C extern
#endif
#endif

/* for valist_CreateDiagnosticAsVoidPtr */
#if defined(__cplusplus)
#include <cstdarg>
#else
#include <stdarg.h>

#endif

#include "slsv_diagnostic_logger_type.h"

#define CODEGEN_SUPPORT_ARG_END_LIST (int)0
#define CODEGEN_SUPPORT_ARG_INTEGER_TYPE (int)1
#define CODEGEN_SUPPORT_ARG_REAL_TYPE (int)2
#define CODEGEN_SUPPORT_ARG_STRING_TYPE (int)3
#define CODEGEN_SUPPORT_ARG_ENCODED_STRING_TYPE (int)4
#define CODEGEN_SUPPORT_ARG_ESCAPED_STRING_TYPE (int)5

#define CODEGEN_SUPPORT_DIAGNOSTIC_INFO (int)0
#define CODEGEN_SUPPORT_DIAGNOSTIC_WARNING (int)1
#define CODEGEN_SUPPORT_DIAGNOSTIC_ERROR (int)2

SL_SERVICES_PUBLISHED_C char* CreateDiagnosticXMLSnippet(const char* id, int nargs, ...);

SL_SERVICES_PUBLISHED_C void* CreateDiagnosticAsVoidPtr(const char* id, int nargs, ...);

SL_SERVICES_PUBLISHED_C void* valist_CreateDiagnosticAsVoidPtr(const char* id,
                                                               int nargs,
                                                               va_list args);

/* UTF8 path */
SL_SERVICES_PUBLISHED_C void dl_set_home(const char* path);

SL_SERVICES_PUBLISHED_C dl_logger_sid_t dl_init(size_t);
SL_SERVICES_PUBLISHED_C dl_logger_sid_t dl_init_objpath(size_t);
SL_SERVICES_PUBLISHED_C void dl_clear(dl_logger_sid_t);
SL_SERVICES_PUBLISHED_C void dl_destroy(dl_logger_sid_t);
SL_SERVICES_PUBLISHED_C void dl_reset(dl_logger_sid_t);

SL_SERVICES_PUBLISHED_C void dl_report(dl_logger_sid_t, int type, const char* id, ...);

/* void * from CreateDiagnosticAsVoidPtr */
SL_SERVICES_PUBLISHED_C void dl_report_from_diagnostic(dl_logger_sid_t, int type, void* diagnostic);

SL_SERVICES_PUBLISHED_C char* dl_get_object_path(dl_logger_sid_t, int sysId, int objId);

SL_SERVICES_PUBLISHED_C void slsvRegisterDiagnosticListenerCG(const char* id, bool enable);
SL_SERVICES_PUBLISHED_C void slsvUnRegisterDiagnosticListenerCG();

SL_SERVICES_PUBLISHED_C void slsvEnableDiagnosticListenerCG();
SL_SERVICES_PUBLISHED_C void slsvDisableDiagnosticListenerCG();

SL_SERVICES_PUBLISHED_C void slsvClearListenerCG(const char* id);
SL_SERVICES_PUBLISHED_C void slsvClearAllListenersCG();

SL_SERVICES_PUBLISHED_C void slsvSetUserDataDiagnosticListenerCG(const void* data);

SL_SERVICES_PUBLISHED_C void slsvTic();
SL_SERVICES_PUBLISHED_C double slsvToc();


/* The following are c apis to the scl/core/crypto/hasher.hpp, add it here
    untile we have a better place
*/

/* Opaque object for C/C++ interface to SCL Hasher Object */
typedef void* HasherPtr;

/* Type of Digest of the hash results */
typedef unsigned char Sha1DigestType[20];
typedef unsigned char Md5DigestType[16];


/* Factory and destructor functions for Sha1Hasher */
SL_SERVICES_PUBLISHED_C HasherPtr ssCreateSha1Hasher(void);
SL_SERVICES_PUBLISHED_C void ssDeleteSha1Hasher(HasherPtr obj);

/*  Process data for computing the hash value */
/*  obj is the pointer to the Hasher
    data is a void pointer of data of interest
    nbytes is the numebr of byte to read
*/
SL_SERVICES_PUBLISHED_C void ssProcessSha1Hasher(HasherPtr obj, const void* data, size_t nbytes);

/* Get the hash value so far */
SL_SERVICES_PUBLISHED_C void ssGetDigestOfSha1Hasher(HasherPtr obj, Sha1DigestType d);



/* Factory and destructor functions for Md5Hasher */
SL_SERVICES_PUBLISHED_C HasherPtr ssCreateMd5Hasher(void);
SL_SERVICES_PUBLISHED_C void ssDeleteMd5Hasher(HasherPtr obj);

/*  Process data for computing the hash value */
/*  obj is the pointer to the Hasher
    data is a void pointer of data of interest
    nbytes is the numebr of byte to read
*/
SL_SERVICES_PUBLISHED_C void ssProcessMd5Hasher(HasherPtr obj, const void* data, size_t nbytes);

/* Get the hash value so far */
SL_SERVICES_PUBLISHED_C void ssGetDigestOfMd5Hasher(HasherPtr obj, Md5DigestType d);

#endif /* slsv_diagnostic_codegen_c_api_h */
