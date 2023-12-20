/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/collated_trajectory_builder.h"

#include "cartographer/common/time.h"
#include "glog/logging.h"

#include <ctime>
#include <chrono>
#include <stdio.h>
const std::string RED("\033[0;31m");
const std::string GREEN("\033[1;32m");
const std::string YELLOW("\033[1;33m");
const std::string CYAN("\033[0;36m");
const std::string MAGENTA("\033[0;35m");
const std::string RESET("\033[0m");
class Timer
{
public:
    Timer(const char *nameIn)
    {
        name = nameIn;
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> dt = end - start;
        return dt.count() * 1000;
    }

    void tic_toc()
    {
        printf("The process %s takes %f ms.\n", name, toc());
    }

private:
    const char* name;
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

int odom_pub_cnt = 0;
double runtime = 0;

namespace cartographer {
namespace mapping {

namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    const proto::TrajectoryBuilderOptions& trajectory_options,
    sensor::CollatorInterface* const sensor_collator, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      collate_landmarks_(trajectory_options.collate_landmarks()),
      collate_fixed_frame_(trajectory_options.collate_fixed_frame()),
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now()) {
  absl::flat_hash_set<std::string> expected_sensor_id_strings;
  for (const auto& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == SensorId::SensorType::LANDMARK &&
        !collate_landmarks_) {
      continue;
    }
    if (sensor_id.type == SensorId::SensorType::FIXED_FRAME_POSE &&
        !collate_fixed_frame_) {
      continue;
    }
    expected_sensor_id_strings.insert(sensor_id.id);
  }
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_id_strings,
      [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });
}

void CollatedTrajectoryBuilder::AddData(std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}

void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
      Timer t_odm("LidarOdometry");
  auto it = rate_timers_.find(sensor_id);
  if (it == rate_timers_.end()) {
    it = rate_timers_
             .emplace(
                 std::piecewise_construct, std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
  }
  it->second.Pulse(data->GetTime());

  if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }

  data->AddToTrajectoryBuilder(wrapped_trajectory_builder_.get());
  std::cout<<RED<<"odom_pub_cnt: "<<++odom_pub_cnt<<RESET<<std::endl;
  t_odm.tic_toc();
  runtime += t_odm.toc();
  std::cout<<RED<<"Odometry ALL run time (ms): "<<runtime<<RESET<<std::endl;//注意这个计数里包含了所有传感器的计数，所以不适合直接除以odom_pub_cnt
  // std::cout<<RED<<"Odometry average run time (ms): "<<runtime / odom_pub_cnt<<RESET<<std::endl;
}

}  // namespace mapping
}  // namespace cartographer
