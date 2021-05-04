/**
 * @file aeb.h
 * @brief Functions to enable and disable AEB bits (debug and test features)
 */

#ifndef __AEB_H
#define __AEB_H

#include "utils/generic/generic.h"
#include "aeb_features.h"

/**
 * @brief Enable Access Enable Block bits
 * 
 * @param uint32_t Mask of AEB bits to enable
 * 
 * @return void
 */
void AebEnable(uint32_t aebMask);

/**
 * @brief Enable Access Enable Block feature
 * 
 * @param aebFeature AEB feature to enable
 * 
 * @return void
 */
void AebEnableFeature(Aeb_Feature_t aebFeature);

/**
 * @brief Disable Access Enable Block bits
 * 
 * @param uint32_t Mask of AEB bits to disable
 * 
 * @return void
 */
void AebDisable(uint32_t aebMask);

/**
 * @brief Disable Access Enable Block Feature
 * 
 * @param aebFeature AEB feature to disable
 * 
 * @return void
 */
void AebDisableFeature(Aeb_Feature_t aebFeature);

/**
 * @brief Restore Access Enable Block bits back to their power-on default
 * 
 * @param uint32_t Mask of AEB bits to restore to default
 * 
 * @return void
 */
void AebRestoreDefault(uint32_t aebMask);

/**
 * @brief Restore Access Enable Block Feature back to its power-on default
 * 
 * @param aebFeature AEB feature to restore to default
 * 
 * @return void
 */
void AebRestoreDefaultFeature(Aeb_Feature_t aebFeature);

/**
 * @brief Lock Access Enable Block bits
 * 
 * @param uint32_t Mask of AEB bits to lock
 * 
 * @return void
 */
void AebLock(uint32_t aebMask);

/**
 * @brief Lock Access Enable Block Feature
 * 
 * @param aebFeature AEB feature to lock
 * 
 * @return void
 */
void AebLockFeature(Aeb_Feature_t aebFeature);

/**
 * @brief Check if AEB is enabled on a given feature
 * 
 * @param aebFeature AEB feature to be checked if enabled
 * 
 * @return bool
 */
bool IsAebFeatureEnabled(Aeb_Feature_t aebFeature);

/**
 * @brief Get current AEB status bits as a single 32-bit word
 * 
 * @param void
 * 
 * @return uint32_t AEB status bits
 */
uint32_t GetAebStatusBits(void);

/**
 * @brief Helper function to check whether an AEB feature enum is valid
 * 
 * @param aebFeature AEB feature
 * 
 * @return bool
 */
static inline bool IsValidAebFeature(Aeb_Feature_t aebFeature)
{
    return (aebFeature >= AEB_FEATURE_FIRST && aebFeature <= AEB_FEATURE_LAST);
}

#endif /* __AEB_H */
