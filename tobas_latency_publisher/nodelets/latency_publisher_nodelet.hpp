#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_latency_publisher/latency_publisher.hpp"

namespace tobas_latency_publisher
{
class LatencyPublisherNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<LatencyPublisher> node_;
};
}  // namespace tobas_latency_publisher
