// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file Num.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _Num_H_
#define _Num_H_

#include <stdint.h>
#include <stdbool.h>

/*!
 * @brief This struct represents the structure Num defined by the user in the IDL file.
 * @ingroup NUM
 */

/*
    We are using typedef for the topics/struct as this would help us properly differentiate the serialization/deserialization functions
    for the different ROS 2 topics.
*/

typedef struct Num
{
    int32_t num;
} Num;

struct ucdrBuffer;

bool Num_serialize_topic(struct ucdrBuffer* writer, const Num* topic);
bool Num_deserialize_topic(struct ucdrBuffer* reader, Num* topic);
uint32_t Num_size_of_topic(const Num* topic, uint32_t size);

#endif // _Num_H_