# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Get the name of a Typesupport target so it can be used to depend on
# generated messages in the same package that generated them.
#
# :param var: A name of a variable to store the typesupport target name
# :param generate_interfaces_target: the target name passed to
#   rosidl_generate_interfaces
# :type generate_interfaces_target: string
# :param typesupport_name: the package name of the type support
# :type typesupport_name: string
#
# @public
#
function(rosidl_get_typesupport_target var generate_interfaces_target typesupport_name)
  if(NOT TARGET ${generate_interfaces_target})
    message(FATAL_ERROR
      "${generate_interfaces_target} is not a CMake target. Maybe rosidl_generate_interfaces was given a different target name?")
  endif()

  set(output_target "${generate_interfaces_target}__${typesupport_name}")

  if(NOT TARGET ${output_target})
    # CMake if() evaluates strings ending in `-NOTFOUND` as false
    set(output_target "${output_target}-NOTFOUND")
  endif()

  set("${var}" "${output_target}" PARENT_SCOPE)
endfunction()