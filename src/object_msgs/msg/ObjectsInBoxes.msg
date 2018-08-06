# Copyright(c) 2018 Senscape Corporation.
# License: Apache 2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# 定义检测到的对象及边框值
std_msgs/Header header        # 原始时间
ObjectInBox[] objects_vector  # 对象数组的结果
float32 inference_time_ms     # 毫秒单位时间戳
