// Copyright 2023 The MathWorks, Inc.

#ifndef ISO_CPP11_STD_MUTEX_H
#define ISO_CPP11_STD_MUTEX_H

#include <mutex>

/** @brief Code replacement function for "RTW_MUTEX_INIT" for mutex 
 *         initialization. This function employs std::mutex available in 
 *         C++11 (same hereinafter).
 */
inline 
void cpp11_std_mutex_init(void** mutexDW){
    *mutexDW = static_cast<void*>(new std::mutex());
}

/** @brief Code replacement function for "RTW_MUTEX_LOCK" for mutex 
 *         lock. 
 */
inline 
void cpp11_std_mutex_lock(void* mutexDW){
    static_cast<std::mutex*>(mutexDW)->lock();
}

/** @brief Code replacement function for "RTW_MUTEX_UNLOCK" for  
 *         mutex unlock. 
 */
inline 
void cpp11_std_mutex_unlock(void* mutexDW){
    static_cast<std::mutex*>(mutexDW)->unlock();
}

/** @brief Code replacement function for "RTW_MUTEX_DESTROY" for  
 *         mutex destroy. 
 */
inline 
void cpp11_std_mutex_destroy(void* mutexDW){
    delete static_cast<std::mutex*>(mutexDW);
}

#endif