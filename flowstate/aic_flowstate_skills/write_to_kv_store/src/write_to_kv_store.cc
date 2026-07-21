/*
 * Copyright (C) 2026 Intrinsic Innovation LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "write_to_kv_store.h"

#include <memory>
#include <string>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/wrappers.pb.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "write_to_kv_store.pb.h"

std::unique_ptr<intrinsic::skills::SkillInterface> WriteToKVStore::CreateSkill() {
  return std::make_unique<WriteToKVStore>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>> WriteToKVStore::Preview(
    const intrinsic::skills::PreviewRequest& /*request*/,
    intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Preview not supported for this skill");
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>> WriteToKVStore::Execute(
    const intrinsic::skills::ExecuteRequest& request,
    intrinsic::skills::ExecuteContext& /*context*/) {
  INTR_ASSIGN_OR_RETURN(
      auto params,
      request.params<ai::flowstate::WriteToKVStoreParams>());

  std::string key = params.key();
  if (key.empty()) {
    LOG(ERROR) << "WriteToKVStore failed: Storage location key is empty.";
    return absl::InvalidArgumentError("Storage location key must not be empty");
  }

  LOG(INFO) << "Executing WriteToKVStore for key: '" << key << "', count: " << params.count();

  // Connect to default PubSub KVStore ("kv_store" prefix)
  intrinsic::PubSub pubsub;
  INTR_ASSIGN_OR_RETURN(intrinsic::KeyValueStore kvstore,
                        pubsub.KeyValueStore());

  google::protobuf::Int64Value value_msg;
  value_msg.set_value(params.count());

  LOG(INFO) << "Setting key '" << key << "' to value " << params.count() << " in KV store with high_consistency=true";

  // Enable high_consistency to block until the value is confirmed persisted in the store
  absl::Status status = kvstore.Set(key, value_msg, /*high_consistency=*/true);
  if (!status.ok()) {
    LOG(ERROR) << "Failed to set key '" << key << "' in KV store: " << status.message();
    return status;
  }

  LOG(INFO) << "Successfully wrote counter value " << params.count() << " to key '" << key << "'";

  auto result = std::make_unique<ai::flowstate::WriteToKVStoreResult>();
  result->set_success(true);
  result->set_message(absl::StrCat("Successfully wrote counter value ", params.count(), " to key '", key, "'"));

  return result;
}
