/**
 * @file dictionary.h
 * 
 * Dictionary structure and related operations. These are
 * used accross the parameter parsing process and passing them 
 * to the application.
 */
#ifndef __DICTIONARY_H
#define __DICTIONARY_H

#include "generic.h"
#include "cparser_config.h"

#ifdef __cplusplus
extern "C"
{
#endif
    /**
 * Dictionary structure.
 */
    typedef struct
    {
        /*@{*/
        char keys[CPARSER_CONFIG_MAX_NUM_OF_PARAMS];    /**< Letters paired with the values(keys) */
        void *values[CPARSER_CONFIG_MAX_NUM_OF_PARAMS]; /**< Pointers to the values */
        uint8_t numberOfElements;                       /**< Number of elements */
        /*@}*/
    } Dictionary_Dictionary_t;

    /**
     * @brief Adds element to the dictionary.
     * 
     * @param dictionary: Pointer to the dictionary object.
     * @param key: Key of the element.
     * @param value: Pointer of the value of element.
     */
    static inline void Dictionary_Add(Dictionary_Dictionary_t *dictionary, char key, void *value)
    {
        dictionary->keys[dictionary->numberOfElements] = key;
        dictionary->values[dictionary->numberOfElements] = value;
        dictionary->numberOfElements++;
    }

    /**
     * @brief Removes element from the dictionary.
     * 
     * @param dictionary: Pointer to the dictionary object.
     * @param key: Key of the element.
     */
    static inline void Dictionary_Remove(Dictionary_Dictionary_t *dictionary, char key)
    {
        // For all elements including the last element.
        for (uint8_t i = 0; i < dictionary->numberOfElements; i++)
        {
            if (dictionary->keys[i] == key)
            {
                // Put last element to the place of the removed element. If the element
                //to be removed is the last element, this procedure would also lead to
                //removing it.
                dictionary->keys[i] = dictionary->keys[dictionary->numberOfElements - 1];
                dictionary->values[i] = dictionary->values[dictionary->numberOfElements - 1];

                // Decrease number of elements.
                dictionary->numberOfElements--;

                return;
            }
        }
    }

    /**
     * @brief Clears all the elements of the dictionary.
     * 
     * @param dictionary: Pointer to the dictionary object.
     */
    static inline void Dictionary_Clear(Dictionary_Dictionary_t *dictionary)
    {
        dictionary->numberOfElements = 0;
    }

    /**
     * @brief Check if the element exists in the given dictionary.
     * 
     * @param dictionary: Pointer to the dictionary object.
     * @param key: Key of the element.
     * 
     * @retval TRUE or FALSE.
     */
    static inline Bool_t Dictionary_DoesExist(Dictionary_Dictionary_t *dictionary, char key)
    {
        for (uint8_t i = 0; i < dictionary->numberOfElements; i++)
        {
            if (dictionary->keys[i] == key)
            {
                return TRUE;
            }
        }

        return FALSE;
    }

    /**
     * @brief Parses pointer of the element value from the dictionary.
     * 
     * @param dictionary: Pointer to the dictionary object.
     * @param key: Key of the element.
     * 
     * @retval NULL or pointer to the element value.
     */
    static inline void *Dictionary_Get(Dictionary_Dictionary_t *dictionary, char key)
    {
        for (uint8_t i = 0; i < dictionary->numberOfElements; i++)
        {
            if (dictionary->keys[i] == key)
            {
                return dictionary->values[i];
            }
        }

        return NULL;
    }

#ifdef __cplusplus
}
#endif

#endif